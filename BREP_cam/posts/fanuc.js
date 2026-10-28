'use strict';
// Generic Fanuc (0i/30i style) milling post for BREP CAM.
// Written from the controller's documented G/M-code conventions; no vendor
// post code was consulted or copied.

const xyz = post.length({ decimals: 3, forceDecimal: true });
const ijk = post.length({ decimals: 3, forceDecimal: true });
const feedFmt = post.feed({ decimals: 1 });
const rpmFmt = post.format({ decimals: 0 });
const secFmt = post.format({ decimals: 3 });
const msFmt = post.format({ decimals: 0 });
const pitchFmt = post.length({ decimals: 4 });
const intFmt = post.format({ decimals: 0 });

const xOut = post.modal('X', xyz), yOut = post.modal('Y', xyz), zOut = post.modal('Z', xyz);
const iOut = post.word('I', ijk), jOut = post.word('J', ijk), kOut = post.word('K', ijk);
const fOut = post.modal('F', feedFmt);
const sOut = post.modal('S', rpmFmt);
const gMotion = post.group();   // G0 G1 G2 G3
const gPlane = post.group();    // G17 G18 G19
const gAbs = post.group();      // G90 G91
const gFeedMode = post.group(); // G94 G95
const gUnits = post.group();    // G20 G21
const gWcs = post.group();      // G54..G59, G54.1 Pn
const gCycle = post.group();    // G80..G89

// Rotary words (one modal per axis in the resolver's joint order) and feed
// words: G93 inverse time with F on every block while active, G94 otherwise.
const rotFmt = post.format({ decimals: 3, forceDecimal: true });
const invFmt = post.format({ decimals: 3 });
const rotOut = {};
const rotaryWords = m => !m.rotary ? [] : Object.keys(m.rotary).map(k => { if (!rotOut[k]) rotOut[k] = post.modal(k, rotFmt); return rotOut[k](m.rotary[k]); });
const feedWords = m => { if (m.feedMode === 'inverseTime') { fOut.reset(); return [gFeedMode('G93'), 'F' + invFmt(m.inverseTime)]; } return [gFeedMode('G94'), fOut(m.feed)]; };

post.define({
  id: 'fanuc',
  name: 'Generic Fanuc',
  description: 'Fanuc 0i/30i style 3-axis and 3+2/5-axis milling (G68.2, G43.4, G93 inverse time): G54-G59 and G54.1 Pn work offsets, G43 length compensation, canned cycles G81/G82/G83/G73/G84/G85/G86/G76/G87, helical G2/G3, G28 return, M19 orient, M29 rigid tapping mode.',
  vendor: 'Fanuc',
  extension: 'nc',
  units: 'auto',
  comment: t => '(' + t.toUpperCase() + ')',
  capabilities: {
    arcs: { planes: ['xy', 'xz', 'yz'], helical: true, maxSweepDeg: 360, minRadius: 0.01 },
    cycles: ['drill', 'peck', 'chipBreak', 'tap', 'ream', 'bore', 'backBore'],
    linearizeTolerance: 0.01,
    multiAxis: { rotary: true, workPlane: true, cycleInPlane: true, tcp: true, feedMode: 'inverseTime' },
  },
  properties: {
    probeVariableBase: { title: 'Probe variable base', description: 'First of the 29 macro variables a probe cycle uses (#base..#base+28): triggers, then the arithmetic', type: 'integer', default: 100, min: 100, max: 170 },
    probeTolerance: { title: 'Probe consistency tolerance', description: 'Alarm when circle contacts disagree by more than this (program units)', type: 'number', default: 0.01, min: 0.000001, max: 1 },
    programNumber: { title: 'Program number', description: 'O-number of the program; the setup program number wins when set', type: 'integer', default: 1001, min: 1, max: 9999 },
    sequenceNumbers: { title: 'Sequence numbers', description: 'Write N numbers', type: 'boolean', default: false },
    sequenceIncrement: { title: 'Sequence increment', type: 'integer', default: 10, min: 1, max: 1000 },
    useG28: { title: 'Return home with G28', description: 'G28 G91 Z0 at tool changes and at the end', type: 'boolean', default: true },
    useG0ForLinks: { title: 'G0 for link moves', description: 'Post stay-down traverse links as G0 instead of G1 at high feed', type: 'boolean', default: false },
    safeStartBlock: { title: 'Safe start block', description: 'G17 G40 G49 G80 G90 at the start of every section', type: 'boolean', default: true },
    optionalStop: { title: 'Optional stop', description: 'M1 before every tool change', type: 'boolean', default: true },
    dwellInMilliseconds: { title: 'Dwell in milliseconds', description: 'G4 P in milliseconds (true) or seconds with X (false)', type: 'boolean', default: true },
  },

  onOpen(ctx) {
    this.sectionCount = 0;
    if (post.property('sequenceNumbers')) post.startSequence(10, post.property('sequenceIncrement'));
    const number = ctx.program.number || post.property('programNumber');
    post.writeln('%');
    post.writeln('O' + String(number).padStart(4, '0') + (ctx.program.name ? ' (' + ctx.program.name.toUpperCase() + ')' : ''));
    if (ctx.program.comment) post.comment(ctx.program.comment);
    for (const t of ctx.program.tools) {
      post.comment('T' + t.number + ' ' + (t.name || t.kind) + ' D=' + xyz(t.geometry.diameter) + (t.geometry.cornerRadius ? ' CR=' + xyz(t.geometry.cornerRadius) : '') + ' L=' + xyz(t.geometry.cuttingLength));
    }
    post.writeBlock('G90', 'G94', 'G17', 'G40', 'G49', 'G80');
    gAbs.set('G90'); gFeedMode.set('G94'); gPlane.set('G17'); gCycle.set('G80');
    post.writeBlock(gUnits(ctx.units === 'in' ? 'G20' : 'G21'));
  },

  onSection(section) {
    post.writeln('');
    post.comment(section.name);
    if (section.toolChange) {
      if (this.sectionCount > 0) {
        if (post.property('useG28')) post.writeBlock('G28', 'G91', 'Z0.');
        if (post.property('optionalStop')) post.writeBlock('M1');
        gAbs.reset();
      }
    }
    this.sectionCount++;
    this.section = section;
    this.lengthOffset = section.lengthOffset;
    this.wcsNumber = section.wcs.number;
  },

  onToolChange(ctx) {
    const t = ctx.tool;
    post.writeBlock('T' + t.number, 'M6');
    post.comment((t.name || t.kind) + ' D=' + xyz(t.geometry.diameter));
    xOut.reset(); yOut.reset(); zOut.reset(); fOut.reset(); gMotion.reset(); gPlane.reset(); sOut.reset();
    this.pendingStart = true;
  },

  onSpindle(s) {
    if (s.on) post.writeBlock(sOut(s.rpm), s.direction === 'ccw' ? 'M4' : 'M3');
    else { post.writeBlock('M5'); sOut.reset(); }
  },

  onSpindleOrient(o) {
    // M19 orients to the parameter-set position; the generic post has no angle word.
    if (Math.abs(o.angleDeg) > 1e-9) throw new Error('spindle orientation to ' + o.angleDeg + ' deg is not supported by the generic Fanuc post (M19 orients to the parameter position only)');
    post.writeBlock('M19');
  },
  onSpindleSync(s) {
    // Rigid tapping mode: M29 S arms it for the tapping that follows; the
    // control leaves it at the next G80 or group-01 G code, so off has no code.
    if (s.enabled) post.writeBlock('M29', 'S' + rpmFmt(s.rpm));
    else post.note('spindleSync off: Fanuc leaves rigid tapping mode at the next G80 or group-01 G code; no code emitted');
  },
  onPolarInterpolation(pi) { post.writeBlock(pi.enabled ? 'G12.1' : 'G13.1'); xOut.reset(); yOut.reset(); },
  onCylindricalInterpolation(ci) { post.writeBlock('G7.1', 'C' + xyz(ci.enabled ? ci.radius : 0)); xOut.reset(); yOut.reset(); zOut.reset(); },
  onCoolant(c) {
    const code = { off: 'M9', flood: 'M8', mist: 'M7', throughTool: 'M8', air: 'M9', floodAndThroughTool: 'M8', floodAndMist: 'M8' }[c.mode] || 'M9';
    post.writeBlock(code);
  },

  startBlock() {
    // Emitted before the first motion of a section: WCS, plane, length offset.
    if (!this.pendingStart) return;
    this.pendingStart = false;
    if (post.property('safeStartBlock')) post.writeBlock(gPlane('G17'), 'G40', 'G49', 'G80', 'G90');
    const n = this.wcsNumber;
    post.writeBlock(gWcs(n >= 1 && n <= 6 ? 'G' + (53 + n) : 'G54.1 P' + (n - 6)));
    this.needLengthOffset = true;
  },

  onRapid(m) {
    this.startBlock();
    if (this.needLengthOffset) {
      // The first rapid after a tool change: XY first, then Z with the length offset.
      post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), rotaryWords(m));
      post.writeBlock('G43', zOut(m.to.z) || 'Z' + xyz(m.to.z), 'H' + intFmt(this.lengthOffset));
      this.needLengthOffset = false;
      return;
    }
    post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), rotaryWords(m));
  },

  onLinear(m) {
    this.startBlock();
    if (m.kind === 'link' && post.property('useG0ForLinks')) { post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z)); return; }
    if (this.needLengthOffset) { post.writeBlock('G43', 'Z' + xyz(post.position().z), 'H' + intFmt(this.lengthOffset)); this.needLengthOffset = false; zOut.set(post.position().z); }
    post.writeBlock(gMotion('G1'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), rotaryWords(m), feedWords(m));
  },

  onCircular(a) {
    this.startBlock();
    const plane = { xy: 'G17', xz: 'G18', yz: 'G19' }[a.plane];
    const code = a.clockwise ? 'G2' : 'G3';
    const words = [gPlane(plane), gMotion(code)];
    // Centre offsets are incremental from the start point.
    const i = a.center.x - a.from.x, j = a.center.y - a.from.y, k = a.center.z - a.from.z;
    words.push(xOut(a.to.x), yOut(a.to.y), zOut(a.to.z));
    if (a.plane === 'xy') words.push(iOut(i), jOut(j));
    else if (a.plane === 'xz') words.push(iOut(i), kOut(k));
    else words.push(jOut(j), kOut(k));
    words.push(...rotaryWords(a), ...feedWords(a));
    post.writeBlock(words);
    gMotion.set(code);
  },

  // ---- multi-axis: the host hands resolved rotary words per block (lane C10's resolver) ----
  onWorkPlane(w) {
    this.startBlock();
    const e = post.eulerZXZ(w.xAxis, w.zAxis);
    post.writeBlock('G68.2', 'X' + xyz(w.origin.x), 'Y' + xyz(w.origin.y), 'Z' + xyz(w.origin.z), 'I' + rotFmt(e.i), 'J' + rotFmt(e.j), 'K' + rotFmt(e.k));
    xOut.reset(); yOut.reset(); zOut.reset();
  },
  onWorkPlaneOff() { post.writeBlock('G69'); xOut.reset(); yOut.reset(); zOut.reset(); },
  onRotaryPosition(r) { this.startBlock(); post.writeBlock(gMotion('G0'), rotaryWords(r)); },
  onTcpMode(t) { if (t.enabled) post.writeBlock('G43.4', 'H' + intFmt(this.lengthOffset)); else post.writeBlock('G49'); zOut.reset(); },


  // Manual NC entries (lane C15): program stop / optional stop, and user text
  // written as a line of its own, byte for byte.
  onStop(s) { post.writeBlock(s.optional ? 'M1' : 'M0'); },
  onPassThrough(t) { post.writeln(t.text); },
  // Controller probing (lane C14's cycles): G31 skip move; a move that reaches its end did not trigger and alarms (#3000); the trigger is read
  // from #5061-#5063 (active work coordinates, length offset applied: the
  // probe tool's G43 H register must hold its calibrated sphere-bottom length),
  // and after the cycle's last point the work offset register is updated with
  // G10 L2 Pn from its stored value plus the measured displacement, the
  // arithmetic of probe::results::evaluate. Angle and inspection cycles refuse.
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
  onDwell(d) {
    if (post.property('dwellInMilliseconds')) post.writeBlock('G4', 'P' + msFmt(d.seconds * 1000));
    else post.writeBlock('G4', 'X' + secFmt(d.seconds));
  },

  onCycle(c) {
    this.startBlock();
    const cy = c.cycle;
    const r = 'R' + xyz(cy.retractZ), z = 'Z' + xyz(cy.depthZ);
    const f = fOut(c.feed);
    const x = xOut(c.at.x), y = yOut(c.at.y);
    let words;
    switch (cy.kind) {
      case 'drill': words = cy.dwellSeconds > 0 ? [gCycle('G82'), x, y, z, r, 'P' + msFmt(cy.dwellSeconds * 1000), f] : [gCycle('G81'), x, y, z, r, f]; break;
      case 'peck': words = [gCycle('G83'), x, y, z, r, 'Q' + xyz(cy.peck), f]; break;
      case 'chipBreak': words = [gCycle('G73'), x, y, z, r, 'Q' + xyz(cy.peck), f]; break;
      case 'tap': {
        const rpm = this.section.spindleRpm;
        const tapFeed = post.feed({ decimals: 2 })(cy.pitch * rpm);
        if (cy.rigid) post.writeBlock('M29', 'S' + rpmFmt(rpm));
        words = [gCycle(cy.leftHand ? 'G74' : 'G84'), x, y, z, r, 'F' + tapFeed];
        fOut.reset();
        break;
      }
      case 'ream': words = [gCycle('G85'), x, y, z, r, f]; break;
      case 'bore':
        if (cy.shift > 0) words = [gCycle('G76'), x, y, z, r, 'Q' + xyz(cy.shift), cy.dwellSeconds > 0 ? 'P' + msFmt(cy.dwellSeconds * 1000) : '', f];
        else if (cy.feedOut) words = [gCycle(cy.dwellSeconds > 0 ? 'G89' : 'G85'), x, y, z, r, cy.dwellSeconds > 0 ? 'P' + msFmt(cy.dwellSeconds * 1000) : '', f];
        else words = [gCycle('G86'), x, y, z, r, cy.dwellSeconds > 0 ? 'P' + msFmt(cy.dwellSeconds * 1000) : '', f];
        break;
      case 'backBore': words = [gCycle('G87'), x, y, z, r, 'Q' + xyz(cy.shift), f]; break;
      default: throw new Error('unsupported cycle ' + cy.kind);
    }
    post.writeBlock(words);
    this.inCycle = true;
  },

  onSectionEnd() {
    if (this.inCycle) { post.writeBlock(gCycle('G80')); this.inCycle = false; }
  },

  onComment(c) { if (!c.isSectionHeader) post.comment(c.text); },
  onClose() {
    if (this.inCycle) { post.writeBlock(gCycle('G80')); this.inCycle = false; }
    if (post.property('useG28')) { post.writeBlock('G28', 'G91', 'Z0.'); post.writeBlock('G28', 'G91', 'X0.', 'Y0.'); post.writeBlock('G90'); }
    post.writeBlock('M30');
    post.writeln('%');
  },
});
