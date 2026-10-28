'use strict';
// GRBL (1.1) post for BREP CAM: routers and hobby mills. No canned cycles,
// no tool length offsets; tool changes are M6 with a pause when enabled.

const xyz = post.length({ decimals: 3 });
const feedFmt = post.feed({ decimals: 0 });
const rpmFmt = post.format({ decimals: 0 });
const secFmt = post.format({ decimals: 2 });
const xOut = post.modal('X', xyz), yOut = post.modal('Y', xyz), zOut = post.modal('Z', xyz);
const iOut = post.word('I', xyz), jOut = post.word('J', xyz), kOut = post.word('K', xyz);
const fOut = post.modal('F', feedFmt), sOut = post.modal('S', rpmFmt);
const gMotion = post.group(), gPlane = post.group(), gWcs = post.group();

post.define({
  id: 'grbl',
  name: 'GRBL',
  description: 'GRBL 1.1 (routers, hobby mills): G54-G59, G0/G1/G2/G3 with helical arcs, no canned cycles (expanded), spindle M3/M4/M5, coolant M7/M8/M9, optional M6 tool change pause.',
  vendor: 'GRBL',
  extension: 'nc',
  units: 'auto',
  capabilities: {
    arcs: { planes: ['xy', 'xz', 'yz'], helical: true, maxSweepDeg: 180, minRadius: 0.01 },
    cycles: [],
    linearizeTolerance: 0.01,
  },
  properties: {
    toolChangePause: { title: 'Pause for tool changes', description: 'Emit M6 then M0 at tool changes (requires a controller that accepts M6)', type: 'boolean', default: true },
    homeAtStart: { title: 'Rapid to Z0 machine at start', type: 'boolean', default: false },
    safeZ: { title: 'Safe Z for tool changes', description: 'Machine Z (G53) the tool rises to before a change', type: 'number', default: 0 },
    useG0ForLinks: { title: 'G0 for link moves', type: 'boolean', default: false },
  },
  onOpen(ctx) {
    this.count = 0;
    post.comment(ctx.program.name);
    if (ctx.program.comment) post.comment(ctx.program.comment);
    for (const t of ctx.program.tools) post.comment('T' + t.number + ' ' + (t.name || t.kind) + ' D=' + xyz(t.geometry.diameter));
    post.writeBlock('G90', 'G94', 'G17', ctx.units === 'in' ? 'G20' : 'G21');
    gPlane.set('G17');
    if (post.property('homeAtStart')) post.writeBlock('G53', 'G0', 'Z' + xyz(post.property('safeZ')));
  },
  onSection(section) {
    post.comment(section.name);
    this.count++;
    this.section = section;
    this.pendingWcs = true;
    this.wcs = section.wcs.number;
  },
  onToolChange(ctx) {
    if (this.count > 1) post.writeBlock('G53', 'G0', 'Z' + xyz(post.property('safeZ')));
    post.comment('tool ' + ctx.tool.number + ' ' + (ctx.tool.name || ctx.tool.kind) + ' D=' + xyz(ctx.tool.geometry.diameter));
    if (post.property('toolChangePause')) { post.writeBlock('T' + ctx.tool.number, 'M6'); post.writeBlock('M0'); }
    xOut.reset(); yOut.reset(); zOut.reset(); fOut.reset(); gMotion.reset(); sOut.reset();
  },
  onSpindle(s) { if (s.on) post.writeBlock(s.direction === 'ccw' ? 'M4' : 'M3', sOut(s.rpm)); else { post.writeBlock('M5'); sOut.reset(); } },
  onSpindleOrient() { throw new Error('GRBL has no spindle orientation (M19); the operation cannot be posted to this controller'); },
  onSpindleSync() { throw new Error('GRBL cannot synchronize the spindle for rigid tapping; use a floating tap (rigid: false)'); },
  onCoolant(c) { post.writeBlock({ off: 'M9', flood: 'M8', mist: 'M7' }[c.mode] || 'M8'); },
  start() {
    if (!this.pendingWcs) return;
    this.pendingWcs = false;
    const n = this.wcs;
    if (n < 1 || n > 6) { post.warn('GRBL has six work offsets; G' + (53 + Math.min(n, 6)) + ' used for offset ' + n); }
    post.writeBlock(gWcs('G' + (53 + Math.min(Math.max(n, 1), 6))));
  },
  onRapid(m) { this.start(); post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z)); },
  onLinear(m) {
    this.start();
    if (m.kind === 'link' && post.property('useG0ForLinks')) { post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z)); return; }
    post.writeBlock(gMotion('G1'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), fOut(m.feed));
  },
  onCircular(a) {
    this.start();
    const w = [gPlane({ xy: 'G17', xz: 'G18', yz: 'G19' }[a.plane]), gMotion(a.clockwise ? 'G2' : 'G3'), xOut(a.to.x), yOut(a.to.y), zOut(a.to.z)];
    const i = a.center.x - a.from.x, j = a.center.y - a.from.y, k = a.center.z - a.from.z;
    if (a.plane === 'xy') w.push(iOut(i), jOut(j)); else if (a.plane === 'xz') w.push(iOut(i), kOut(k)); else w.push(jOut(j), kOut(k));
    w.push(fOut(a.feed));
    post.writeBlock(w);
  },
  // Manual NC entries (lane C15): program stop / optional stop, and user text
  // written as a line of its own, byte for byte.
  onStop(s) { post.writeBlock(s.optional ? 'M1' : 'M0'); },
  onPassThrough(t) { post.writeln(t.text); },
  onDwell(d) { post.writeBlock('G4', 'P' + secFmt(d.seconds)); },
  onComment(c) { if (!c.isSectionHeader) post.comment(c.text); },
  onClose() { post.writeBlock('G53', 'G0', 'Z' + xyz(post.property('safeZ'))); post.writeBlock('M30'); },
});
