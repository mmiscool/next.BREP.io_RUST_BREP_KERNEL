'use strict';
// Siemens SINUMERIK 840D milling post for BREP CAM. Written from the
// SINUMERIK programming manual; no vendor post code consulted or copied.

const xyz = post.length({ decimals: 3 });
const feedFmt = post.feed({ decimals: 1 });
const rpmFmt = post.format({ decimals: 0 });
const secFmt = post.format({ decimals: 3 });
const intFmt = post.format({ decimals: 0 });
const xOut = post.modal('X', xyz), yOut = post.modal('Y', xyz), zOut = post.modal('Z', xyz);
const iOut = post.word('I', xyz), jOut = post.word('J', xyz), kOut = post.word('K', xyz);
const fOut = post.modal('F', feedFmt), sOut = post.modal('S', rpmFmt);
const gMotion = post.group(), gPlane = post.group(), gWcs = post.group(), gFeedMode = post.group();

const rotFmt = post.format({ decimals: 3, forceDecimal: true });
const invFmt = post.format({ decimals: 3 });
const rotOut = {};
const rotaryWords = m => !m.rotary ? [] : Object.keys(m.rotary).map(k => { if (!rotOut[k]) rotOut[k] = post.modal(k, rotFmt); return rotOut[k](m.rotary[k]); });
const feedWords = m => { if (m.feedMode === 'inverseTime') { fOut.reset(); return [gFeedMode('G93'), 'F' + invFmt(m.inverseTime)]; } return [gFeedMode('G94'), fOut(m.feed)]; };

post.define({
  id: 'siemens-840d',
  name: 'Siemens SINUMERIK 840D',
  description: 'SINUMERIK 840D/828D milling: G54-G57 and G505+ work offsets, T/M6/D length offset, CYCLE81/82/83/84/840/85/86 drilling with MCALL, helical arcs, G4 F dwell.',
  vendor: 'Siemens',
  extension: 'mpf',
  units: 'auto',
  comment: t => '; ' + t,
  capabilities: {
    arcs: { planes: ['xy', 'xz', 'yz'], helical: true, maxSweepDeg: 360, minRadius: 0.01 },
    cycles: ['drill', 'peck', 'chipBreak', 'tap', 'ream', 'bore'],
    // CYCLE800 needs the machine's swivel data set; the generic post writes rotary words, TRAORI and G93 only.
    multiAxis: { rotary: true, workPlane: false, tcp: true, feedMode: 'inverseTime' },
    linearizeTolerance: 0.01,
  },
  properties: {
    probeVariableBase: { title: 'Probe variable base', description: 'First of the 29 variables (Rbase..Rbase+28) a probe cycle uses: triggers, then the arithmetic', type: 'integer', default: 50, min: 0, max: 71 },
    probeTolerance: { title: 'Probe consistency tolerance', description: 'Alarm when circle contacts disagree by more than this (program units)', type: 'number', default: 0.01, min: 0.000001, max: 1 },
    sequenceNumbers: { title: 'Block numbers', type: 'boolean', default: true },
    optionalStop: { title: 'Optional stop (M1) before tool changes', type: 'boolean', default: true },
    useG0ForLinks: { title: 'G0 for link moves', type: 'boolean', default: false },
    toolNameCall: { title: 'Call tools by name', description: 'T="name" instead of T number', type: 'boolean', default: false },
  },
  onOpen(ctx) {
    this.count = 0;
    if (post.property('sequenceNumbers')) post.startSequence(10, 10);
    post.comment(ctx.program.name);
    if (ctx.program.comment) post.comment(ctx.program.comment);
    for (const t of ctx.program.tools) post.comment('T' + t.number + ' ' + (t.name || t.kind) + ' D=' + xyz(t.geometry.diameter));
    post.writeBlock('G17', 'G40', 'G90', 'G94', ctx.units === 'in' ? 'G70' : 'G71');
    gFeedMode.set('G94');
    gPlane.set('G17');
  },
  onSection(section) {
    post.comment(section.name);
    if (section.toolChange && this.count > 0) { post.writeBlock('SUPA', 'G0', 'Z0', 'D0'); if (post.property('optionalStop')) post.writeBlock('M1'); }
    this.count++;
    this.section = section;
    this.pending = true;
    this.wcs = section.wcs.number;
  },
  onToolChange(ctx) {
    post.writeBlock(post.property('toolNameCall') ? 'T="' + (ctx.tool.name || 'T' + ctx.tool.number) + '"' : 'T' + ctx.tool.number, 'M6');
    post.writeBlock('D' + intFmt(ctx.section.lengthOffset));
    xOut.reset(); yOut.reset(); zOut.reset(); fOut.reset(); gMotion.reset(); sOut.reset();
  },
  onSpindle(s) { if (s.on) post.writeBlock(sOut(s.rpm), s.direction === 'ccw' ? 'M4' : 'M3'); else { post.writeBlock('M5'); sOut.reset(); } },
  onSpindleOrient(o) { post.writeBlock('SPOS=' + post.format({ decimals: 3 })(o.angleDeg)); },
  onSpindleSync(s) { post.note('spindleSync ' + (s.enabled ? 'on' : 'off') + ': SINUMERIK CYCLE84 synchronizes the spindle itself; no code emitted'); },
  onPolarInterpolation(pi) { post.writeBlock(pi.enabled ? 'TRANSMIT' : 'TRAFOOF'); xOut.reset(); yOut.reset(); },
  onCylindricalInterpolation(ci) { post.writeBlock(ci.enabled ? 'TRACYL(' + xyz(2 * ci.radius) + ')' : 'TRAFOOF'); xOut.reset(); yOut.reset(); zOut.reset(); },
  onCoolant(c) { post.writeBlock({ off: 'M9', flood: 'M8', mist: 'M7', floodAndMist: 'M7 M8' }[c.mode] || 'M8'); },
  start() {
    if (!this.pending) return;
    this.pending = false;
    const n = this.wcs;
    post.writeBlock(gPlane('G17'), gWcs(n >= 1 && n <= 4 ? 'G' + (53 + n) : 'G' + (500 + n)));
  },
  onRapid(m) { this.start(); post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), rotaryWords(m)); },
  onLinear(m) {
    this.start();
    if (m.kind === 'link' && post.property('useG0ForLinks')) { post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), rotaryWords(m)); return; }
    post.writeBlock(gMotion('G1'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), rotaryWords(m), feedWords(m));
  },
  onRotaryPosition(r) { this.start(); post.writeBlock(gMotion('G0'), rotaryWords(r)); },
  onTcpMode(t) { post.writeBlock(t.enabled ? 'TRAORI' : 'TRAFOOF'); xOut.reset(); yOut.reset(); zOut.reset(); },
  onCircular(a) {
    this.start();
    const w = [gPlane({ xy: 'G17', xz: 'G18', yz: 'G19' }[a.plane]), gMotion(a.clockwise ? 'G2' : 'G3'), xOut(a.to.x), yOut(a.to.y), zOut(a.to.z)];
    // SINUMERIK takes incremental centre offsets I/J/K.
    const i = a.center.x - a.from.x, j = a.center.y - a.from.y, k = a.center.z - a.from.z;
    if (a.plane === 'xy') w.push(iOut(i), jOut(j)); else if (a.plane === 'xz') w.push(iOut(i), kOut(k)); else w.push(jOut(j), kOut(k));
    if (a.helical) w.push('TURN=0');
    w.push(...rotaryWords(a), ...feedWords(a));
    post.writeBlock(w);
  },
  // Manual NC entries (lane C15): program stop / optional stop, and user text
  // written as a line of its own, byte for byte.
  onStop(s) { post.writeBlock(s.optional ? 'M1' : 'M0'); },
  onPassThrough(t) { post.writeln(t.text); },
  // Controller probing (lane C14's cycles): MEAS=1 G1 measuring move then STOPRE; $AC_MEA[1] = 0 (no trigger) raises SETAL; the trigger is read from $AA_MW[X/Y/Z] and the update adds the displacement to $P_UIFR[n,axis,TR], re-selected with the frame's G code. The arithmetic is
  // probe::results::evaluate's (post.probeMacro). The probe tool's length
  // offset must hold its calibrated sphere-bottom length. Angle and
  // inspection cycles refuse.
  onProbe(p) {
    const m = post.probeMacro(p, { dialect: 'siemens', base: post.property('probeVariableBase'), eps: post.property('probeTolerance'), maxWcs: 99, endFormat: xyz, noTriggerBand: 0 });
    const put = b => (typeof b === 'string' ? post.writeBlock(b) : post.writeln(b.raw));
    if (p.isFirst) post.comment('PROBE ' + p.cycle.kind + ' WCS ' + p.cycle.wcs.number + ' (' + p.cycle.pointCount + ' POINTS)');
    const e = p.overtravel, a = p.approach;
    post.writeBlock('MEAS=1', 'G1', 'X' + xyz(e.x), 'Y' + xyz(e.y), 'Z' + xyz(e.z), 'F' + feedFmt(p.feed));
    post.writeBlock('STOPRE');
    m.point.forEach(put);
    xOut.reset(); yOut.reset(); zOut.reset(); fOut.reset(); gMotion.reset();
    if (p.isLast) {
      // Back to the approach point in the old frame, then the update.
      post.writeBlock(gMotion('G1'), xOut(a.x), yOut(a.y), zOut(a.z), fOut(p.feed));
      m.update.forEach(put);
      xOut.reset(); yOut.reset(); zOut.reset(); fOut.reset(); gMotion.reset();
      this.pending = true; gWcs.reset();
    }
  },
  onDwell(d) { post.writeBlock('G4', 'F' + secFmt(d.seconds)); },
  onCycle(c) {
    this.start();
    const cy = c.cycle;
    const rtp = xyz(cy.retractZ), rfp = xyz(c.at.z), sdis = xyz(Math.max(0, cy.retractZ - c.at.z)), dp = xyz(cy.depthZ);
    post.writeBlock(gMotion('G0'), xOut(c.at.x), yOut(c.at.y));
    gMotion.reset();
    const f = feedFmt(c.feed);
    let call;
    switch (cy.kind) {
      case 'drill': call = 'CYCLE82(' + [rtp, rfp, sdis, dp, '0', secFmt(cy.dwellSeconds || 0)].join(',') + ')'; break;
      case 'peck': call = 'CYCLE83(' + [rtp, rfp, sdis, dp, '0', xyz(cy.depthZ + Math.max(0, c.at.z - cy.depthZ) - Math.min(cy.peck, Math.abs(c.at.z - cy.depthZ))), '0', xyz(cy.peck), '0', '0', '1', '1', f, f, '0', '0', '0'].join(',') + ')'; break;
      case 'chipBreak': call = 'CYCLE83(' + [rtp, rfp, sdis, dp, '0', xyz(cy.depthZ + Math.max(0, c.at.z - cy.depthZ) - Math.min(cy.peck, Math.abs(c.at.z - cy.depthZ))), '0', xyz(cy.peck), '0', '0', '1', '0', f, f, '0', '0', xyz(cy.backOff || 0.5)].join(',') + ')'; break;
      case 'tap': call = cy.rigid ? 'CYCLE84(' + [rtp, rfp, sdis, dp, '0', '0', cy.leftHand ? '4' : '3', '0', xyz(cy.pitch), '0', rpmFmt(this.section.spindleRpm), rpmFmt(this.section.spindleRpm)].join(',') + ')' : 'CYCLE840(' + [rtp, rfp, sdis, dp, '0', '0', '0', '0', cy.leftHand ? '4' : '3', '0', '0', xyz(cy.pitch)].join(',') + ')'; break;
      case 'ream': call = 'CYCLE85(' + [rtp, rfp, sdis, dp, '0', '0', f, f].join(',') + ')'; break;
      case 'bore': call = cy.feedOut ? 'CYCLE85(' + [rtp, rfp, sdis, dp, '0', secFmt(cy.dwellSeconds || 0), f, f].join(',') + ')' : 'CYCLE86(' + [rtp, rfp, sdis, dp, '0', secFmt(cy.dwellSeconds || 0), '3', '0', xyz(cy.shift || 0), '0', '0'].join(',') + ')'; break;
      default: throw new Error('unsupported cycle ' + cy.kind);
    }
    post.writeBlock('F' + f, call);
    fOut.set(c.feed);
    zOut.set(cy.retractZ);
  },
  onComment(c) { if (!c.isSectionHeader) post.comment(c.text); },
  onClose() { post.writeBlock('SUPA', 'G0', 'Z0', 'D0'); post.writeBlock('M30'); },
});
