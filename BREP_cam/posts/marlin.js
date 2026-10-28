'use strict';
// Marlin / RepRap-flavoured post for BREP CAM: routers and engravers running
// 3D-printer firmware (G0/G1/G2/G3, M3/M5 spindle or laser, no cycles).

const xyz = post.length({ decimals: 3 });
const feedFmt = post.feed({ decimals: 0 });
const pwrFmt = post.format({ decimals: 0 });
const msFmt = post.format({ decimals: 0 });
const xOut = post.modal('X', xyz), yOut = post.modal('Y', xyz), zOut = post.modal('Z', xyz);
const iOut = post.word('I', xyz), jOut = post.word('J', xyz);
const fOut = post.modal('F', feedFmt);
const fRapid = post.modal('F', post.format({ decimals: 0 }));
const gMotion = post.group();

post.define({
  id: 'marlin',
  name: 'Marlin / RepRap router',
  description: 'Marlin 2.x and RepRap firmware on routers and engravers: G0/G1, XY arcs G2/G3, M3/M5 spindle (S as rpm or 0-255 PWM), G4 P dwell in milliseconds, no work offsets or cycles (expanded).',
  vendor: 'Marlin',
  extension: 'gcode',
  units: 'auto',
  comment: t => '; ' + t,
  capabilities: {
    arcs: { planes: ['xy'], helical: true, maxSweepDeg: 180, minRadius: 0.05 },
    cycles: [],
    linearizeTolerance: 0.02,
  },
  properties: {
    spindlePwm: { title: 'Spindle S as PWM (0-255)', description: 'Scale rpm to 0-255 by the maximum rpm property', type: 'boolean', default: false },
    maxRpm: { title: 'Maximum spindle rpm', type: 'number', default: 12000, min: 1 },
    toolChangePause: { title: 'Pause for tool changes (M0)', type: 'boolean', default: true },
    rapidFeed: { title: 'Rapid feed (units/min)', description: 'G0 moves carry this F; Marlin ignores feed on G0 only when configured', type: 'number', default: 3000, min: 1 },
  },
  onOpen(ctx) {
    post.comment(ctx.program.name);
    if (ctx.program.comment) post.comment(ctx.program.comment);
    for (const t of ctx.program.tools) post.comment('T' + t.number + ' ' + (t.name || t.kind) + ' D=' + xyz(t.geometry.diameter));
    post.writeBlock(ctx.units === 'in' ? 'G20' : 'G21');
    post.writeBlock('G90');
    post.writeBlock('G17');
    this.count = 0;
  },
  onSection(section) { post.comment(section.name); this.count++; if (section.wcs.number !== 1 && !this.warned) { post.warn('Marlin has no work offsets; every setup posts in machine coordinates'); this.warned = true; } },
  onToolChange(ctx) {
    post.comment('tool ' + ctx.tool.number + ' ' + (ctx.tool.name || ctx.tool.kind) + ' D=' + xyz(ctx.tool.geometry.diameter));
    if (this.count > 1 && post.property('toolChangePause')) post.writeBlock('M0', 'Change to T' + ctx.tool.number);
    xOut.reset(); yOut.reset(); zOut.reset(); fOut.reset(); gMotion.reset();
  },
  onSpindle(s) {
    if (!s.on) { post.writeBlock('M5'); return; }
    const value = post.property('spindlePwm') ? Math.min(255, Math.round(255 * s.rpm / post.property('maxRpm'))) : s.rpm;
    post.writeBlock(s.direction === 'ccw' ? 'M4' : 'M3', 'S' + pwrFmt(value));
  },
  onSpindleOrient() { throw new Error('Marlin has no spindle orientation (M19); the operation cannot be posted to this controller'); },
  onSpindleSync() { throw new Error('Marlin cannot synchronize the spindle for rigid tapping; use a floating tap (rigid: false)'); },
  onCoolant(c) { post.writeBlock(c.mode === 'off' ? 'M9' : c.mode === 'mist' ? 'M7' : 'M8'); },
  onRapid(m) {
    // The rapid feed property is already in the post's units.
    post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), fRapid(post.property('rapidFeed')));
    fOut.reset();
  },
  onLinear(m) { post.writeBlock(gMotion('G1'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), fOut(m.feed)); fRapid.reset(); },
  onCircular(a) {
    post.writeBlock(gMotion(a.clockwise ? 'G2' : 'G3'), xOut(a.to.x), yOut(a.to.y), zOut(a.to.z), iOut(a.center.x - a.from.x), jOut(a.center.y - a.from.y), fOut(a.feed));
  },
  // Manual NC entries (lane C15). Marlin's M1 is an unconditional stop (an
  // alias of M0), so an optional stop is refused rather than made mandatory.
  onStop(s) {
    if (s.optional) throw new Error('optional stop: Marlin has no optional stop (its M1 is an unconditional stop like M0); use a program stop');
    post.writeBlock('M0');
  },
  onPassThrough(t) { post.writeln(t.text); },
  onDwell(d) { post.writeBlock('G4', 'P' + msFmt(d.seconds * 1000)); },
  onComment(c) { if (!c.isSectionHeader) post.comment(c.text); },
  onClose() { post.writeBlock('M5'); post.writeBlock('M9'); post.writeBlock('M30'); },
});
