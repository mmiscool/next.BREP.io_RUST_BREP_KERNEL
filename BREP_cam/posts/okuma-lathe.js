'use strict';
// Okuma OSP lathe post (LB/LU series) for BREP CAM: diameter X with radial
// incremental I/K, G94 feed per minute, G96/G97 spindle with the G50 S limit,
// G33 synchronized threading, Toooooo tool calls (offset, turret station,
// nose radius number), G30 Pn home. Written from the documented OSP
// conventions; no vendor post code consulted or copied, not run on a
// machine. OSP lathes have no plane selection: the XZ plane is fixed.

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
  id: 'okuma-lathe',
  name: 'Okuma OSP lathe',
  description: 'Okuma OSP-P lathes, 2 axes: X as diameter (the program radius doubled by the post) with radial incremental I/K, G94 feed per minute, G97 S / G96 S with G50 S limit, G33 synchronized threading (a canned thread schedule is expanded to G33 passes), Toooooo tool calls, G30 Pn home.',
  vendor: 'Okuma',
  machine: 'lathe',
  extension: 'min',
  units: 'auto',
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
    homePosition: { title: 'Home position (G30 Pn)', description: 'Home position number for tool changes and the program end', type: 'integer', default: 1, min: 1, max: 4 },
  },
  onOpen(ctx) {
    this.count = 0;
    if (post.property('sequenceNumbers')) post.startSequence(10, 10);
    const number = ctx.program.number || post.property('programNumber');
    post.writeln('O' + String(number).padStart(4, '0') + (ctx.program.name ? ' (' + ctx.program.name.toUpperCase() + ')' : ''));
    if (ctx.program.comment) post.comment(ctx.program.comment);
    for (const t of ctx.program.tools) post.comment('T' + t.number + ' ' + (t.name || t.kind));
    post.writeBlock('G40', 'G80', 'G94');
    post.writeBlock(ctx.units === 'in' ? 'G20' : 'G21');
  },

  onSection(section) {
    post.writeln('');
    post.comment(section.name);
    if (section.toolChange && this.count > 0) {
      post.writeBlock('G30', 'P' + intFmt(post.property('homePosition')));
      if (post.property('optionalStop')) post.writeBlock('M1');
    }
    this.count++;
    this.section = section;
  },
  onToolChange(ctx) {
    // Toooooo: tool offset number, turret station, nose radius number.
    post.writeBlock('T' + two(ctx.section.lengthOffset) + two(ctx.tool.number) + two(ctx.section.lengthOffset));
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
  onSpindleSync() { throw new Error('the lathe post has no rigid tapping mode; synchronized threading is G33'); },
  onLatheMode(l) {
    // X words are diameters when the operation asks for it; I/K stay radial.
    this.dia = !!l.diameterMode;
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
    post.writeBlock(gMotion(code), xOut(this.xw(a.to.x)), zOut(a.to.z), iOut(a.center.x - a.from.x), kOut(a.center.z - a.from.z), fOut(a.feed));
    gMotion.set(code);
  },
  onThreadCut(t) {
    // Synchronized constant-pitch threading: F is the lead, never a feed rate.
    this.check(t.to, 'threadCut');
    post.writeBlock('G33', xOut(this.xw(t.to.x)), zOut(t.to.z), 'F' + pitchFmt(t.pitch));
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
    post.writeBlock('G30', 'P' + intFmt(post.property('homePosition')));
    post.writeBlock('M30');
  },
});
