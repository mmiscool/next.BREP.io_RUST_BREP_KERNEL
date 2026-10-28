'use strict';
// Generic Fanuc-style lathe post for BREP CAM (G18, diameter X, G98 feed per
// minute, G96/G50/G97 spindle, G32 synchronized threading). Written from the
// documented conventions; no vendor post code consulted or copied.
//
// Live-tool milling (mill-turn, G12.1 / G13.1 polar interpolation only). The
// C-axis engage/release and live-spindle codes, the address of the polar
// hypothetical axis and how X is read in G12.1 are set by the machine builder,
// so each is a property with no default, and an unset one refuses by name.

const xyz = post.length({ decimals: 3, forceDecimal: true });
const feedFmt = post.feed({ decimals: 1 });
const pitchFmt = post.length({ decimals: 4 });
const rpmFmt = post.format({ decimals: 0 });
const cssFmt = post.format({ decimals: 1 });
const intFmt = post.format({ decimals: 0 });
const xOut = post.modal('X', xyz), zOut = post.modal('Z', xyz);
let vOut = null; // the polar hypothetical axis (its address is a property)
const iOut = post.word('I', xyz), kOut = post.word('K', xyz);
const fOut = post.modal('F', feedFmt), sOut = post.modal('S', rpmFmt);
const gMotion = post.group(), gPlane = post.group(), gSpindleMode = post.group();
const two = n => String(n).padStart(2, '0');
const msFmt = post.format({ decimals: 0 });

post.define({
  id: 'fanuc-lathe',
  name: 'Generic Fanuc lathe',
  description: 'Fanuc-style 2-axis lathe: G18, X as diameter (the program radius doubled by the post) with radial I/K, G98 feed per minute, G97 S / G96 S with G50 S limit, G32 synchronized threading (a canned thread schedule is expanded to G32 passes), Tnnoo tool calls, G28 U0 W0 home.',
  vendor: 'Fanuc',
  machine: 'lathe',
  extension: 'nc',
  units: 'auto',
  comment: t => '(' + t.toUpperCase() + ')',
  capabilities: {
    arcs: { planes: ['xz'], helical: false, maxSweepDeg: 360, minRadius: 0.01 },
    cycles: [],
    turnThreadCycle: false,
    linearizeTolerance: 0.01,
    // C indexing inside a live-tool section only (onLiveTooling); linear
    // moves never carry a changing rotary word.
    multiAxis: { rotary: true },
  },
  properties: {
    programNumber: { title: 'Program number', type: 'integer', default: 1001, min: 1, max: 9999 },
    sequenceNumbers: { title: 'Sequence numbers', type: 'boolean', default: false },
    cAxisEngage: { title: 'C-axis engage block', type: 'string', default: '', description: 'The block that switches the main spindle to C-axis (rotary) mode on this machine, e.g. an M-code from the builder\'s manual. Empty refuses live-tool milling.' },
    cAxisRelease: { title: 'C-axis release block', type: 'string', default: '', description: 'The block that returns the main spindle to turning mode. Empty refuses live-tool milling.' },
    liveToolForward: { title: 'Live tool forward block', type: 'string', default: '', description: 'Starts the live tool clockwise; {rpm} is replaced by the speed (e.g. the builder\'s M-code with its speed word). Empty refuses live-tool milling.' },
    liveToolReverse: { title: 'Live tool reverse block', type: 'string', default: '', description: 'Starts the live tool counter-clockwise; {rpm} is replaced by the speed. Empty refuses counter-clockwise live tools.' },
    liveToolStop: { title: 'Live tool stop block', type: 'string', default: '', description: 'Stops the live tool. Empty refuses live-tool milling.' },
    polarAxis: { title: 'Polar hypothetical axis address', type: 'enum', values: ['unset', 'C', 'Y', 'H'], default: 'unset', description: 'The address G12.1 programs the hypothetical axis with on this control (a parameter). Unset refuses polar milling.' },
    polarXMode: { title: 'X inside G12.1', type: 'enum', values: ['unset', 'diameter', 'radius'], default: 'unset', description: 'How this control reads X while G12.1 is active. Unset refuses polar milling.' },
    optionalStop: { title: 'Optional stop (M1) before tool changes', type: 'boolean', default: true },
  },
  onOpen(ctx) {
    this.count = 0;
    if (post.property('sequenceNumbers')) post.startSequence(10, 10);
    const number = ctx.program.number || post.property('programNumber');
    post.writeln('%');
    post.writeln('O' + String(number).padStart(4, '0') + (ctx.program.name ? ' (' + ctx.program.name.toUpperCase() + ')' : ''));
    if (ctx.program.comment) post.comment(ctx.program.comment);
    for (const t of ctx.program.tools) post.comment('T' + t.number + ' ' + (t.name || t.kind));
    post.writeBlock('G40', 'G80', 'G98');
    post.writeBlock(ctx.units === 'in' ? 'G20' : 'G21');
  },

  onSection(section) {
    post.writeln('');
    post.comment(section.name);
    if (section.toolChange && this.count > 0) {
      post.writeBlock('G28', 'U0.', 'W0.');
      if (post.property('optionalStop')) post.writeBlock('M1');
    }
    this.count++;
    this.section = section;
  },
  onToolChange(ctx) {
    // Tnnoo: tool number and its offset register.
    post.writeBlock('T' + two(ctx.tool.number) + two(ctx.section.lengthOffset));
    post.comment((ctx.tool.name || ctx.tool.kind));
    xOut.reset(); zOut.reset(); fOut.reset(); gMotion.reset(); sOut.reset();
  },
  onSpindle(s) {
    if (s.on) post.writeBlock(gSpindleMode('G97'), sOut(s.rpm), s.direction === 'ccw' ? 'M4' : 'M3'); else { post.writeBlock('M5'); sOut.reset(); }
  },
  onSpindleSurfaceSpeed(c) {
    // Constant surface speed with its rpm clamp; S is m/min (ft/min on inch posts).
    post.writeBlock('G50', 'S' + rpmFmt(c.maxRpm));
    post.writeBlock(gSpindleMode('G96'), 'S' + cssFmt(c.surfaceSpeed), c.direction === 'ccw' ? 'M4' : 'M3');
    sOut.reset();
  },
  onSpindleOrient() { throw new Error('the lathe post has no spindle orientation (C axis) output'); },
  onSpindleSync() { throw new Error('the lathe post has no rigid tapping mode; synchronized threading is G32'); },
  onLatheMode(l) {
    // X words are diameters when the operation asks for it; I/K stay radial.
    this.dia = !!l.diameterMode;
    post.writeBlock(gPlane('G18'));
  },
  // X: inside polar interpolation as the polarXMode property says; elsewhere
  // as the last lathe mode asked, and as a diameter (this post's convention)
  // in a live-tool section that no lathe mode set.
  xw(v) { const dia = this.polar ? post.property('polarXMode') === 'diameter' : (this.dia === undefined ? !!this.live : this.dia); return dia ? 2 * v : v; },
  // A milling move (Y off the spindle axis) cannot post on a lathe: name the
  // motion and where it goes, like every other refusal. Inside G12.1 the
  // programmed Y is the hypothetical axis.
  check(p, kind) { if (!this.polar && Math.abs(p.y) > 1e-9) throw new Error('a lathe program has no Y axis: ' + kind + ' to X' + p.x + ' Y' + p.y + ' Z' + p.z + ' (section "' + (this.section ? this.section.name : '?') + '")'); },
  yw(p) { return this.polar ? vOut(p.y) : ''; },
  rotary(m, kind) { if (m.rotaryChanged) throw new Error('fanuc-lathe moves C only by indexing (rotaryPosition) inside live tooling, never with a ' + kind + ' (section "' + (this.section ? this.section.name : '?') + '")'); },
  onRotaryPosition(r) {
    if (!this.live) throw new Error('fanuc-lathe: C indexing outside a live-tool section (section "' + (this.section ? this.section.name : '?') + '")');
    if (this.polar) throw new Error('fanuc-lathe: C indexing inside polar interpolation (section "' + (this.section ? this.section.name : '?') + '")');
    const words = r.axes.map(a => { if (a.name !== 'C') throw new Error('fanuc-lathe indexes only the C axis, not ' + a.name); return 'C' + xyz(a.angleDeg); });
    post.writeBlock(gMotion('G0'), ...words);
  },
  onRapid(m) { this.rotary(m, 'rapid'); this.check(m.to, 'rapid'); post.writeBlock(gMotion('G0'), xOut(this.xw(m.to.x)), this.yw(m.to), zOut(m.to.z)); },
  onLinear(m) { this.rotary(m, m.kind || 'cut'); this.check(m.to, m.kind || 'cut'); post.writeBlock(gMotion('G1'), xOut(this.xw(m.to.x)), this.yw(m.to), zOut(m.to.z), fOut(m.feed)); },
  // Turning -> live-tool milling and back, with the builder's blocks. In:
  // main spindle stop (M5), C-axis engage, XY plane (G17), live tool start.
  // Out: live tool stop, C-axis release, XZ plane (G18); the next turning
  // section starts its own spindle.
  block(name, rpm) {
    const text = post.property(name);
    return rpm === undefined ? text : text.split('{rpm}').join(rpmFmt(rpm));
  },
  onLiveTooling(l) {
    const where = ' (section "' + l.section.name + '")';
    if (l.enabled) {
      const other = l.interpolation.filter(k => k !== 'polarInterpolation' && k !== 'rotaryPosition');
      if (other.length) throw new Error('fanuc-lathe posts live tooling as C indexing and G12.1 polar interpolation only; ' + other.join(', ') + ' is not posted' + where);
      const polar = l.interpolation.includes('polarInterpolation');
      const start = l.direction === 'ccw' ? 'liveToolReverse' : 'liveToolForward';
      const missing = ['cAxisEngage', 'cAxisRelease', start, 'liveToolStop'].filter(n => !post.property(n).trim());
      if (polar && post.property('polarAxis') === 'unset') missing.push('polarAxis');
      if (polar && post.property('polarXMode') === 'unset') missing.push('polarXMode');
      if (missing.length) throw new Error('fanuc-lathe: set the ' + missing.join(', ') + ' propert' + (missing.length === 1 ? 'y' : 'ies') + ' (the machine builder\'s mill-turn codes) before posting live tooling' + where);
      if (!(l.rpm > 0)) throw new Error('fanuc-lathe: a live tool needs a spindle speed' + where);
      vOut = polar ? post.modal(post.property('polarAxis'), xyz) : null;
      this.live = true;
      post.writeBlock('M5');
      post.writeln(this.block('cAxisEngage'));
      post.writeBlock(gPlane('G17'));
      post.writeln(this.block(start, l.rpm));
    } else {
      if (this.polar) throw new Error('fanuc-lathe: G12.1 is still active at the end of a live-tool section' + where);
      post.writeln(this.block('liveToolStop'));
      post.writeln(this.block('cAxisRelease'));
      post.writeBlock(gPlane('G18'));
      this.live = false;
      sOut.reset(); gSpindleMode.reset();
    }
    xOut.reset(); zOut.reset(); gMotion.reset(); fOut.reset();
    if (vOut) vOut.reset();
  },
  onPolarInterpolation(p) {
    post.writeBlock(p.enabled ? 'G12.1' : 'G13.1');
    this.polar = !!p.enabled;
    xOut.reset(); zOut.reset(); gMotion.reset();
    if (vOut) vOut.reset();
  },
  onCircular(a) {
    if (this.polar) throw new Error('fanuc-lathe does not post arcs inside G12.1 (' + (a.clockwise ? 'arcCw' : 'arcCcw') + ' in section "' + (this.section ? this.section.name : '?') + '"): post the polar section without arc fitting');
    if (a.plane !== 'xz') throw new Error('a lathe arc must lie in the XZ plane');
    this.check(a.to, a.clockwise ? 'arcCw' : 'arcCcw');
    const code = a.clockwise ? 'G2' : 'G3';
    post.writeBlock(gPlane('G18'), gMotion(code), xOut(this.xw(a.to.x)), zOut(a.to.z), iOut(a.center.x - a.from.x), kOut(a.center.z - a.from.z), fOut(a.feed));
    gMotion.set(code);
  },
  onThreadCut(t) {
    // Synchronized constant-pitch threading: F is the lead, never a feed rate.
    this.check(t.to, 'threadCut');
    post.writeBlock('G32', xOut(this.xw(t.to.x)), zOut(t.to.z), 'F' + pitchFmt(t.pitch));
    gMotion.reset(); fOut.reset();
  },
  onCoolant(c) { post.writeBlock(c.mode === 'off' ? 'M9' : 'M8'); },
  onSectionEnd() {},
  onComment(c) { if (!c.isSectionHeader) post.comment(c.text); },
  // Manual NC entries (lane C15): program stop / optional stop, and user text
  // written as a line of its own, byte for byte.
  onStop(s) { post.writeBlock(s.optional ? 'M1' : 'M0'); },
  onPassThrough(t) { post.writeln(t.text); },
  onDwell(d) { post.writeBlock('G4', 'P' + msFmt(d.seconds * 1000)); },
  onClose() {
    post.writeBlock('G28', 'U0.', 'W0.');
    post.writeBlock('M30');
    post.writeln('%');
  },
});
