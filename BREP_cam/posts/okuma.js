'use strict';
// Okuma OSP-P milling post for BREP CAM. Written from the documented OSP
// conventions (G15 H work coordinates, G56 H length offset, absolute arc
// centres, G71 cycle return level, G284/G274 synchronous tapping); no vendor
// post code was consulted or copied.

const xyz = post.length({ decimals: 3, forceDecimal: true });
const feedFmt = post.feed({ decimals: 1 });
const rpmFmt = post.format({ decimals: 0 });
const secFmt = post.format({ decimals: 2 });
const intFmt = post.format({ decimals: 0 });
const xOut = post.modal('X', xyz), yOut = post.modal('Y', xyz), zOut = post.modal('Z', xyz);
const iOut = post.word('I', xyz), jOut = post.word('J', xyz), kOut = post.word('K', xyz);
const fOut = post.modal('F', feedFmt), sOut = post.modal('S', rpmFmt);
const gMotion = post.group(), gPlane = post.group(), gAbs = post.group(), gWcs = post.group(), gCycle = post.group();

post.define({
  id: 'okuma',
  name: 'Okuma OSP',
  description: 'Okuma OSP-P machining centres: G15 Hn work coordinates, G56 H length offset, absolute I/J/K arc centres, G71 return level with G81/G82/G83/G73/G84/G74/G284/G274/G85/G86/G89 cycles, G30 home, M19 orient.',
  vendor: 'Okuma',
  extension: 'min',
  units: 'auto',
  capabilities: {
    arcs: { planes: ['xy', 'xz', 'yz'], helical: true, maxSweepDeg: 360, minRadius: 0.01 },
    cycles: ['drill', 'peck', 'chipBreak', 'tap', 'ream', 'bore'],
    linearizeTolerance: 0.01,
  },
  properties: {
    probeVariableBase: { title: 'Probe variable base', description: 'First of the 29 variables (VCbase..VCbase+28) a probe cycle uses: triggers, then the arithmetic', type: 'integer', default: 100, min: 1, max: 172 },
    probeTolerance: { title: 'Probe consistency tolerance', description: 'Alarm when circle contacts disagree by more than this (program units)', type: 'number', default: 0.01, min: 0.000001, max: 1 },
    programNumber: { title: 'Program number', description: 'O-number of the program; the setup program number wins when set', type: 'integer', default: 1001, min: 1, max: 9999 },
    sequenceNumbers: { title: 'Sequence numbers', type: 'boolean', default: false },
    sequenceIncrement: { title: 'Sequence increment', type: 'integer', default: 10, min: 1, max: 1000 },
    optionalStop: { title: 'Optional stop (M1) before tool changes', type: 'boolean', default: true },
    homePosition: { title: 'Home position (G30 Pn)', description: 'Home position number for tool changes and the program end', type: 'integer', default: 1, min: 1, max: 4 },
    incrementalArcCenters: { title: 'Incremental arc centres', description: 'I/J/K relative to the arc start (parameter-dependent; OSP default is absolute)', type: 'boolean', default: false },
    useG0ForLinks: { title: 'G0 for link moves', type: 'boolean', default: false },
  },
  onOpen(ctx) {
    this.count = 0;
    if (post.property('sequenceNumbers')) post.startSequence(10, post.property('sequenceIncrement'));
    const number = ctx.program.number || post.property('programNumber');
    post.writeln('O' + String(number).padStart(4, '0') + (ctx.program.name ? ' (' + ctx.program.name.toUpperCase() + ')' : ''));
    if (ctx.program.comment) post.comment(ctx.program.comment);
    for (const t of ctx.program.tools) post.comment('T' + t.number + ' ' + (t.name || t.kind) + ' D=' + xyz(t.geometry.diameter));
    post.writeBlock('G0', 'G17', 'G40', 'G80', 'G90', 'G94');
    gMotion.set('G0'); gPlane.set('G17'); gAbs.set('G90'); gCycle.set('G80');
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
    this.pendingStart = this.pendingStart || section.toolChange || this.wcs !== section.wcs.number;
    this.wcs = section.wcs.number;
    this.returnLevel = null;
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
    // M19 orients to the parameter-set position; there is no angle word here.
    if (Math.abs(o.angleDeg) > 1e-9) throw new Error('spindle orientation to ' + o.angleDeg + ' deg is not supported by the Okuma post (M19 orients to the parameter position only)');
    post.writeBlock('M19');
  },
  onSpindleSync(s) { post.note('spindleSync ' + (s.enabled ? 'on' : 'off') + ': Okuma synchronous tapping is the G284/G274 cycle itself; no code emitted'); },
  onCoolant(c) { post.writeBlock(c.mode === 'off' ? 'M9' : c.mode === 'mist' ? 'M7' : 'M8'); },
  start() {
    if (!this.pendingStart) return;
    this.pendingStart = false;
    const n = this.wcs;
    if (n < 1 || n > 20) throw new Error('Okuma OSP work coordinates are G15 H1-H20; H' + n + ' is out of range');
    post.writeBlock(gPlane('G17'), gAbs('G90'), gWcs('G15 H' + intFmt(n)));
  },
  onRapid(m) {
    this.start();
    if (this.needH) {
      post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y));
      post.writeBlock('G56', zOut(m.to.z) || 'Z' + xyz(m.to.z), 'H' + intFmt(this.section.lengthOffset));
      this.needH = false;
      return;
    }
    post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z));
  },
  onLinear(m) {
    this.start();
    if (m.kind === 'link' && post.property('useG0ForLinks')) { post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z)); return; }
    if (this.needH) { post.writeBlock('G56', 'Z' + xyz(post.position().z), 'H' + intFmt(this.section.lengthOffset)); this.needH = false; zOut.set(post.position().z); }
    post.writeBlock(gMotion('G1'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), fOut(m.feed));
  },
  onCircular(a) {
    this.start();
    const code = a.clockwise ? 'G2' : 'G3';
    const w = [gPlane({ xy: 'G17', xz: 'G18', yz: 'G19' }[a.plane]), gMotion(code), xOut(a.to.x), yOut(a.to.y), zOut(a.to.z)];
    const inc = post.property('incrementalArcCenters');
    const i = inc ? a.center.x - a.from.x : a.center.x, j = inc ? a.center.y - a.from.y : a.center.y, k = inc ? a.center.z - a.from.z : a.center.z;
    if (a.plane === 'xy') w.push(iOut(i), jOut(j)); else if (a.plane === 'xz') w.push(iOut(i), kOut(k)); else w.push(jOut(j), kOut(k));
    w.push(fOut(a.feed));
    post.writeBlock(w);
  },
  // Manual NC entries (lane C15): program stop / optional stop, and user text
  // written as a line of its own, byte for byte.
  onStop(s) { post.writeBlock(s.optional ? 'M1' : 'M0'); },
  onPassThrough(t) { post.writeln(t.text); },
  // Controller probing (lane C14's cycles): G31 skip move; a move that reaches its end did not trigger and raises a user alarm (VUACM/VDOUT[992]); the trigger is read from VSKPX/VSKPY/VSKPZ and the update adds the displacement to VZOFX/Y/Z[n], re-selected with G15 Hn. The arithmetic is
  // probe::results::evaluate's (post.probeMacro). The probe tool's length
  // offset must hold its calibrated sphere-bottom length. Angle and
  // inspection cycles refuse.
  onProbe(p) {
    const m = post.probeMacro(p, { dialect: 'okuma', base: post.property('probeVariableBase'), eps: post.property('probeTolerance'), maxWcs: 20, endFormat: xyz, noTriggerBand: 0.0005 });
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
      this.pendingStart = true; gWcs.reset();
    }
  },
  onDwell(d) { post.writeBlock('G4', 'F' + secFmt(d.seconds)); },
  onCycle(c) {
    this.start();
    const cy = c.cycle;
    // The return level is modal (G71 Z); the cycle's R word is the point R.
    if (this.returnLevel !== cy.retractZ) { post.writeBlock('G71', 'Z' + xyz(cy.retractZ)); this.returnLevel = cy.retractZ; }
    const r = 'R' + xyz(cy.retractZ), z = 'Z' + xyz(cy.depthZ), x = xOut(c.at.x), y = yOut(c.at.y), f = fOut(c.feed);
    const dwell = cy.dwellSeconds > 0 ? 'P' + secFmt(cy.dwellSeconds) : '';
    let w;
    switch (cy.kind) {
      case 'drill': w = cy.dwellSeconds > 0 ? [gCycle('G82'), x, y, z, r, dwell, f] : [gCycle('G81'), x, y, z, r, f]; break;
      case 'peck': w = [gCycle('G83'), x, y, z, r, 'Q' + xyz(cy.peck), f]; break;
      case 'chipBreak': w = [gCycle('G73'), x, y, z, r, 'Q' + xyz(cy.peck), f]; break;
      case 'tap': {
        const code = cy.rigid ? (cy.leftHand ? 'G274' : 'G284') : (cy.leftHand ? 'G74' : 'G84');
        w = [gCycle(code), x, y, z, r, 'F' + post.feed({ decimals: 2 })(cy.pitch * this.section.spindleRpm)];
        fOut.reset();
        break;
      }
      case 'ream': w = [gCycle('G85'), x, y, z, r, f]; break;
      case 'bore':
        if (cy.shift > 0) throw new Error('the Okuma post has no fine boring cycle with a shift; use a boring cycle without shift or expand the operation');
        if (cy.feedOut) w = [gCycle(cy.dwellSeconds > 0 ? 'G89' : 'G85'), x, y, z, r, dwell, f];
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
    post.writeBlock('G30', 'P' + intFmt(post.property('homePosition')));
    post.writeBlock('M30');
  },
});
