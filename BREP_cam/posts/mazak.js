'use strict';
// Mazak (Matrix / Smooth, EIA/ISO programs) milling post for BREP CAM.
// Written from the documented EIA conventions (G54-G59 and G54.1 Pn, G43 H,
// G84.2/G84.3 synchronous tapping, G61.1 geometry compensation); no vendor
// post code was consulted or copied.

const xyz = post.length({ decimals: 3, forceDecimal: true });
const feedFmt = post.feed({ decimals: 1 });
const rpmFmt = post.format({ decimals: 0 });
const msFmt = post.format({ decimals: 0 });
const intFmt = post.format({ decimals: 0 });
const xOut = post.modal('X', xyz), yOut = post.modal('Y', xyz), zOut = post.modal('Z', xyz);
const iOut = post.word('I', xyz), jOut = post.word('J', xyz), kOut = post.word('K', xyz);
const fOut = post.modal('F', feedFmt), sOut = post.modal('S', rpmFmt);
const gMotion = post.group(), gPlane = post.group(), gAbs = post.group(), gWcs = post.group(), gCycle = post.group(), gFeedMode = post.group();

// Rotary words (one modal per axis in the resolver's joint order) and feed
// words: G93 inverse time with F on every block while active, G94 otherwise.
const rotFmt = post.format({ decimals: 3, forceDecimal: true });
const invFmt = post.format({ decimals: 3 });
const rotOut = {};
const rotaryWords = m => !m.rotary ? [] : Object.keys(m.rotary).map(k => { if (!rotOut[k]) rotOut[k] = post.modal(k, rotFmt); return rotOut[k](m.rotary[k]); });
const feedWords = m => { if (m.feedMode === 'inverseTime') { fOut.reset(); return [gFeedMode('G93'), 'F' + invFmt(m.inverseTime)]; } return [gFeedMode('G94'), fOut(m.feed)]; };

post.define({
  id: 'mazak',
  name: 'Mazak EIA/ISO',
  description: 'Mazak Matrix/Smooth EIA/ISO programs: G54-G59 and G54.1 Pn work offsets, G43 H length offset, G81/G82/G83/G73/G84/G74/G84.2/G84.3/G85/G86/G89/G76 cycles, helical arcs, optional G61.1 geometry compensation, G28 return, M19 orient.',
  vendor: 'Mazak',
  extension: 'eia',
  units: 'auto',
  capabilities: {
    arcs: { planes: ['xy', 'xz', 'yz'], helical: true, maxSweepDeg: 360, minRadius: 0.01 },
    cycles: ['drill', 'peck', 'chipBreak', 'tap', 'ream', 'bore'],
    linearizeTolerance: 0.01,
    multiAxis: { rotary: true, workPlane: true, cycleInPlane: true, tcp: true, feedMode: 'inverseTime' },
  },
  properties: {
    probeVariableBase: { title: 'Probe variable base', description: 'First of the 29 variables (#base..#base+28) a probe cycle uses: triggers, then the arithmetic', type: 'integer', default: 100, min: 100, max: 170 },
    probeTolerance: { title: 'Probe consistency tolerance', description: 'Alarm when circle contacts disagree by more than this (program units)', type: 'number', default: 0.01, min: 0.000001, max: 1 },
    programNumber: { title: 'Program number', description: 'O-number of the program; the setup program number wins when set', type: 'integer', default: 1001, min: 1, max: 99999999 },
    sequenceNumbers: { title: 'Sequence numbers', type: 'boolean', default: false },
    sequenceIncrement: { title: 'Sequence increment', type: 'integer', default: 10, min: 1, max: 1000 },
    optionalStop: { title: 'Optional stop (M1) before tool changes', type: 'boolean', default: true },
    geometryCompensation: { title: 'Geometry compensation (G61.1)', description: 'G61.1 at the start of every cutting section, G64 before tool changes', type: 'boolean', default: false },
    useG28: { title: 'Return home with G28', type: 'boolean', default: true },
    useG0ForLinks: { title: 'G0 for link moves', type: 'boolean', default: false },
  },
  onOpen(ctx) {
    this.count = 0;
    if (post.property('sequenceNumbers')) post.startSequence(10, post.property('sequenceIncrement'));
    const number = ctx.program.number || post.property('programNumber');
    post.writeln('%');
    post.writeln('O' + String(number).padStart(4, '0') + (ctx.program.name ? ' (' + ctx.program.name.toUpperCase() + ')' : ''));
    if (ctx.program.comment) post.comment(ctx.program.comment);
    for (const t of ctx.program.tools) post.comment('T' + t.number + ' ' + (t.name || t.kind) + ' D=' + xyz(t.geometry.diameter));
    post.writeBlock('G0', 'G17', 'G40', 'G49', 'G80', 'G90', 'G94');
    gMotion.set('G0'); gPlane.set('G17'); gAbs.set('G90'); gCycle.set('G80'); gFeedMode.set('G94');
    post.writeBlock(ctx.units === 'in' ? 'G20' : 'G21');
  },
  onSection(section) {
    post.writeln('');
    post.comment(section.name);
    if (section.toolChange && this.count > 0) {
      if (post.property('geometryCompensation')) post.writeBlock('G64');
      if (post.property('useG28')) post.writeBlock('G91', 'G28', 'Z0.');
      gAbs.reset();
      if (post.property('optionalStop')) post.writeBlock('M1');
    }
    this.count++;
    this.section = section;
    this.pendingStart = this.pendingStart || section.toolChange || this.wcs !== section.wcs.number;
    this.wcs = section.wcs.number;
  },
  onToolChange(ctx) {
    post.writeBlock('T' + intFmt(ctx.tool.number), 'M6');
    post.comment((ctx.tool.name || ctx.tool.kind) + ' D=' + xyz(ctx.tool.geometry.diameter));
    xOut.reset(); yOut.reset(); zOut.reset(); fOut.reset(); gMotion.reset(); sOut.reset();
    this.needH = true;
  },
  onSpindle(s) {
    if (s.on) post.writeBlock(sOut(s.rpm), s.direction === 'ccw' ? 'M4' : 'M3'); else { post.writeBlock('M5'); sOut.reset(); }
  },
  onSpindleOrient(o) {
    if (Math.abs(o.angleDeg) > 1e-9) throw new Error('spindle orientation to ' + o.angleDeg + ' deg is not supported by the Mazak post (M19 orients to the parameter position only)');
    post.writeBlock('M19');
  },
  onSpindleSync(s) { post.note('spindleSync ' + (s.enabled ? 'on' : 'off') + ': Mazak synchronous tapping is the G84.2/G84.3 cycle itself; no code emitted'); },
  onPolarInterpolation(pi) { post.writeBlock(pi.enabled ? 'G12.1' : 'G13.1'); xOut.reset(); yOut.reset(); },
  onCylindricalInterpolation(ci) { post.writeBlock('G7.1', 'C' + xyz(ci.enabled ? ci.radius : 0)); xOut.reset(); yOut.reset(); zOut.reset(); },
  onCoolant(c) { post.writeBlock(c.mode === 'off' ? 'M9' : c.mode === 'mist' ? 'M7' : 'M8'); },
  start() {
    if (!this.pendingStart) return;
    this.pendingStart = false;
    const n = this.wcs;
    post.writeBlock(gPlane('G17'), gAbs('G90'), gWcs(n >= 1 && n <= 6 ? 'G' + (53 + n) : 'G54.1 P' + intFmt(n - 6)));
    if (post.property('geometryCompensation')) post.writeBlock('G61.1');
  },
  onRapid(m) {
    this.start();
    if (this.needH) {
      post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), rotaryWords(m));
      post.writeBlock('G43', zOut(m.to.z) || 'Z' + xyz(m.to.z), 'H' + intFmt(this.section.lengthOffset));
      this.needH = false;
      return;
    }
    post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), rotaryWords(m));
  },
  onLinear(m) {
    this.start();
    if (m.kind === 'link' && post.property('useG0ForLinks')) { post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z)); return; }
    if (this.needH) { post.writeBlock('G43', 'Z' + xyz(post.position().z), 'H' + intFmt(this.section.lengthOffset)); this.needH = false; zOut.set(post.position().z); }
    post.writeBlock(gMotion('G1'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), rotaryWords(m), feedWords(m));
  },
  onCircular(a) {
    this.start();
    const code = a.clockwise ? 'G2' : 'G3';
    const w = [gPlane({ xy: 'G17', xz: 'G18', yz: 'G19' }[a.plane]), gMotion(code), xOut(a.to.x), yOut(a.to.y), zOut(a.to.z)];
    const i = a.center.x - a.from.x, j = a.center.y - a.from.y, k = a.center.z - a.from.z;
    if (a.plane === 'xy') w.push(iOut(i), jOut(j)); else if (a.plane === 'xz') w.push(iOut(i), kOut(k)); else w.push(jOut(j), kOut(k));
    w.push(...rotaryWords(a), ...feedWords(a));
    post.writeBlock(w);
  },
  // ---- multi-axis: the host hands resolved rotary words per block (lane C10's resolver) ----
  onWorkPlane(w) {
    this.start();
    const e = post.eulerZXZ(w.xAxis, w.zAxis);
    post.writeBlock('G68.2', 'X' + xyz(w.origin.x), 'Y' + xyz(w.origin.y), 'Z' + xyz(w.origin.z), 'I' + rotFmt(e.i), 'J' + rotFmt(e.j), 'K' + rotFmt(e.k));
    xOut.reset(); yOut.reset(); zOut.reset();
  },
  onWorkPlaneOff() { post.writeBlock('G69'); xOut.reset(); yOut.reset(); zOut.reset(); },
  onRotaryPosition(r) { this.start(); post.writeBlock(gMotion('G0'), rotaryWords(r)); },
  onTcpMode(t) { if (t.enabled) post.writeBlock('G43.4', 'H' + intFmt(this.section.lengthOffset)); else post.writeBlock('G49'); zOut.reset(); },
  // Manual NC entries (lane C15): program stop / optional stop, and user text
  // written as a line of its own, byte for byte.
  onStop(s) { post.writeBlock(s.optional ? 'M1' : 'M0'); },
  onPassThrough(t) { post.writeln(t.text); },
  // Controller probing (lane C14's cycles): G31 skip move; a move that reaches its end did not trigger and alarms (#3000); the trigger is read from #5061-#5063 and the update is G10 L2 Pn from #5221 + 20 (n - 1) + axis (Mazak EIA's Fanuc-compatible macro variables). The arithmetic is
  // probe::results::evaluate's (post.probeMacro). The probe tool's length
  // offset must hold its calibrated sphere-bottom length. Angle and
  // inspection cycles refuse.
  onProbe(p) {
    const m = post.probeMacro(p, { dialect: 'fanuc', base: post.property('probeVariableBase'), eps: post.property('probeTolerance'), maxWcs: 6, endFormat: xyz, noTriggerBand: 0.0005 });
    const put = b => (typeof b === 'string' ? post.writeBlock(b) : post.writeln(b.raw));
    if (p.isFirst) post.comment('PROBE ' + p.cycle.kind + ' WCS ' + p.cycle.wcs.number + ' (' + p.cycle.pointCount + ' POINTS)');
    const e = p.overtravel, a = p.approach;
    post.writeBlock('G31', 'X' + xyz(e.x), 'Y' + xyz(e.y), 'Z' + xyz(e.z), 'F' + feedFmt(p.feed));
    m.point.forEach(put);
    xOut.reset(); yOut.reset(); zOut.reset(); fOut.reset(); gMotion.reset();
    if (p.isLast) {
      // Back to the approach point in the old frame, then the update.
      post.writeBlock(gMotion('G1'), xOut(a.x), yOut(a.y), zOut(a.z), fOut(p.feed));
      m.update.forEach(put);
      xOut.reset(); yOut.reset(); zOut.reset(); fOut.reset(); gMotion.reset();
    }
  },
  onDwell(d) { post.writeBlock('G4', 'P' + msFmt(d.seconds * 1000)); },
  onCycle(c) {
    this.start();
    const cy = c.cycle, r = 'R' + xyz(cy.retractZ), z = 'Z' + xyz(cy.depthZ), x = xOut(c.at.x), y = yOut(c.at.y), f = fOut(c.feed);
    const dwell = cy.dwellSeconds > 0 ? 'P' + msFmt(cy.dwellSeconds * 1000) : '';
    let w;
    switch (cy.kind) {
      case 'drill': w = cy.dwellSeconds > 0 ? [gCycle('G82'), x, y, z, r, dwell, f] : [gCycle('G81'), x, y, z, r, f]; break;
      case 'peck': w = [gCycle('G83'), x, y, z, r, 'Q' + xyz(cy.peck), f]; break;
      case 'chipBreak': w = [gCycle('G73'), x, y, z, r, 'Q' + xyz(cy.peck), f]; break;
      case 'tap': {
        const code = cy.rigid ? (cy.leftHand ? 'G84.3' : 'G84.2') : (cy.leftHand ? 'G74' : 'G84');
        w = [gCycle(code), x, y, z, r, 'F' + post.feed({ decimals: 2 })(cy.pitch * this.section.spindleRpm)];
        fOut.reset();
        break;
      }
      case 'ream': w = [gCycle('G85'), x, y, z, r, f]; break;
      case 'bore':
        if (cy.shift > 0) w = [gCycle('G76'), x, y, z, r, 'Q' + xyz(cy.shift), dwell, f];
        else if (cy.feedOut) w = [gCycle(cy.dwellSeconds > 0 ? 'G89' : 'G85'), x, y, z, r, dwell, f];
        else w = [gCycle('G86'), x, y, z, r, dwell, f];
        break;
      default: throw new Error('unsupported cycle ' + cy.kind);
    }
    post.writeBlock(w);
    this.inCycle = true;
  },
  onSectionEnd() { if (this.inCycle) { post.writeBlock(gCycle('G80')); this.inCycle = false; } },
  onComment(c) { if (!c.isSectionHeader) post.comment(c.text); },
  onClose() {
    if (this.inCycle) post.writeBlock(gCycle('G80'));
    if (post.property('geometryCompensation')) post.writeBlock('G64');
    if (post.property('useG28')) { post.writeBlock('G91', 'G28', 'Z0.'); post.writeBlock('G28', 'X0.', 'Y0.'); post.writeBlock('G90'); }
    post.writeBlock('M30');
    post.writeln('%');
  },
});
