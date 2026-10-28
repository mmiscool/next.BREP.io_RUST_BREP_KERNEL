'use strict';
// Mazak lathe post (Quick Turn / Integrex turning in EIA/ISO mode) for BREP
// CAM: G18, diameter X, G98 feed per minute, G96/G50/G97 spindle, G32
// synchronized threading, Tnnoo, G28 U0 W0 home. Written from the documented
// EIA conventions; no vendor post code consulted or copied, not run on a
// machine.

const xyz = post.length({ decimals: 3, forceDecimal: true });
const feedFmt = post.feed({ decimals: 1 });
const pitchFmt = post.length({ decimals: 4 });
const rpmFmt = post.format({ decimals: 0 });
const cssFmt = post.format({ decimals: 1 });
const intFmt = post.format({ decimals: 0 });
const xOut = post.modal('X', xyz), zOut = post.modal('Z', xyz);
const iOut = post.word('I', xyz), kOut = post.word('K', xyz);
const fOut = post.modal('F', feedFmt), sOut = post.modal('S', rpmFmt);
const gMotion = post.group(), gPlane = post.group(), gSpindleMode = post.group();
const two = n => String(n).padStart(2, '0');
const msFmt = post.format({ decimals: 0 });

post.define({
  id: 'mazak-lathe',
  name: 'Mazak lathe (EIA)',
  description: 'Mazak turning centres in EIA/ISO mode, 2 axes: G18, X as diameter (the program radius doubled by the post) with radial I/K, G98 feed per minute, G97 S / G96 S with G50 S limit, G32 synchronized threading (a canned thread schedule is expanded to G32 passes), Tnnoo tool calls, G28 U0 W0 home.',
  vendor: 'Mazak',
  machine: 'lathe',
  extension: 'eia',
  units: 'auto',
  comment: t => '(' + t.toUpperCase() + ')',
  capabilities: {
    arcs: { planes: ['xz'], helical: false, maxSweepDeg: 360, minRadius: 0.01 },
    cycles: [],
    turnThreadCycle: false,
    linearizeTolerance: 0.01,
  },
  properties: {
    programNumber: { title: 'Program number', type: 'integer', default: 1001, min: 1, max: 9999 },
    sequenceNumbers: { title: 'Sequence numbers', type: 'boolean', default: false },
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
  xw(v) { return this.dia ? 2 * v : v; },
  // A milling move (Y off the spindle axis) cannot post on a lathe: name the
  // motion and where it goes, like every other refusal.
  check(p, kind) { if (Math.abs(p.y) > 1e-9) throw new Error('a lathe program has no Y axis: ' + kind + ' to X' + p.x + ' Y' + p.y + ' Z' + p.z + ' (section "' + (this.section ? this.section.name : '?') + '")'); },
  onRapid(m) { this.check(m.to, 'rapid'); post.writeBlock(gMotion('G0'), xOut(this.xw(m.to.x)), zOut(m.to.z)); },
  onLinear(m) { this.check(m.to, m.kind || 'cut'); post.writeBlock(gMotion('G1'), xOut(this.xw(m.to.x)), zOut(m.to.z), fOut(m.feed)); },
  onCircular(a) {
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
