'use strict';
// LinuxCNC (2.8+) milling post for BREP CAM. Written from the LinuxCNC
// G-code reference; no vendor post code consulted or copied.

const xyz = post.length({ decimals: 4 });
const feedFmt = post.feed({ decimals: 1 });
const rpmFmt = post.format({ decimals: 0 });
const secFmt = post.format({ decimals: 3 });
const intFmt = post.format({ decimals: 0 });
const xOut = post.modal('X', xyz), yOut = post.modal('Y', xyz), zOut = post.modal('Z', xyz);
const iOut = post.word('I', xyz), jOut = post.word('J', xyz), kOut = post.word('K', xyz);
const fOut = post.modal('F', feedFmt), sOut = post.modal('S', rpmFmt);
const gMotion = post.group(), gPlane = post.group(), gWcs = post.group(), gCycle = post.group(), gFeedMode = post.group();

const rotFmt = post.format({ decimals: 3, forceDecimal: true });
const invFmt = post.format({ decimals: 3 });
const rotOut = {};
const rotaryWords = m => !m.rotary ? [] : Object.keys(m.rotary).map(k => { if (!rotOut[k]) rotOut[k] = post.modal(k, rotFmt); return rotOut[k](m.rotary[k]); });
const feedWords = m => { if (m.feedMode === 'inverseTime') { fOut.reset(); return [gFeedMode('G93'), 'F' + invFmt(m.inverseTime)]; } return [gFeedMode('G94'), fOut(m.feed)]; };

post.define({
  id: 'linuxcnc',
  name: 'LinuxCNC',
  description: 'LinuxCNC: G54-G59 and G59.1-G59.3 work offsets, G43 length offset, G64 P path blending, G81/G82/G83/G73/G85/G86/G89 cycles, G33.1 rigid tapping, helical arcs.',
  vendor: 'LinuxCNC',
  extension: 'ngc',
  units: 'auto',
  capabilities: {
    arcs: { planes: ['xy', 'xz', 'yz'], helical: true, maxSweepDeg: 360, minRadius: 0.01 },
    cycles: ['drill', 'peck', 'chipBreak', 'tap', 'ream', 'bore'],
    multiAxis: { rotary: true, workPlane: false, tcp: false, feedMode: 'inverseTime' },
    linearizeTolerance: 0.01,
  },
  properties: {
    probeVariableBase: { title: 'Probe variable base', description: 'First of the 29 macro variables a probe cycle uses (#base..#base+28): triggers, then the arithmetic', type: 'integer', default: 100, min: 31, max: 4900 },
    probeTolerance: { title: 'Probe consistency tolerance', description: 'Alarm when circle contacts disagree by more than this (program units)', type: 'number', default: 0.01, min: 0.000001, max: 1 },
    pathBlending: { title: 'Path blending tolerance (G64 P)', description: '0 disables G64 P', type: 'number', default: 0.01, min: 0 },
    optionalStop: { title: 'Optional stop before tool changes', type: 'boolean', default: true },
    useG0ForLinks: { title: 'G0 for link moves', type: 'boolean', default: false },
    sequenceNumbers: { title: 'Sequence numbers', type: 'boolean', default: false },
  },
  onOpen(ctx) {
    this.count = 0;
    if (post.property('sequenceNumbers')) post.startSequence(10, 10);
    post.writeln('%');
    post.comment(ctx.program.name);
    if (ctx.program.comment) post.comment(ctx.program.comment);
    for (const t of ctx.program.tools) post.comment('T' + t.number + ' ' + (t.name || t.kind) + ' D=' + xyz(t.geometry.diameter));
    post.writeBlock('G17', 'G40', 'G49', 'G80', 'G90', 'G94', ctx.units === 'in' ? 'G20' : 'G21');
    gFeedMode.set('G94');
    gPlane.set('G17'); gCycle.set('G80');
    const p = post.property('pathBlending');
    if (p > 0) post.writeBlock('G64', 'P' + xyz(p));
  },
  onSection(section) {
    post.writeln('');
    post.comment(section.name);
    if (section.toolChange && this.count > 0) { post.writeBlock('G53', 'G0', 'Z0'); if (post.property('optionalStop')) post.writeBlock('M1'); }
    this.count++;
    this.section = section;
    this.pending = true;
    this.wcs = section.wcs.number;
  },
  onToolChange(ctx) {
    post.writeBlock('T' + ctx.tool.number, 'M6', 'G43', 'H' + intFmt(ctx.section.lengthOffset));
    post.comment((ctx.tool.name || ctx.tool.kind) + ' D=' + xyz(ctx.tool.geometry.diameter));
    xOut.reset(); yOut.reset(); zOut.reset(); fOut.reset(); gMotion.reset(); sOut.reset();
  },
  onSpindle(s) { if (s.on) post.writeBlock(sOut(s.rpm), s.direction === 'ccw' ? 'M4' : 'M3'); else { post.writeBlock('M5'); sOut.reset(); } },
  onSpindleOrient(o) { post.writeBlock('M19', 'R' + post.format({ decimals: 3 })(o.angleDeg)); },
  onSpindleSync(s) { post.note('spindleSync ' + (s.enabled ? 'on' : 'off') + ': LinuxCNC synchronizes the spindle inside G33.1; no code emitted'); },
  onCoolant(c) { post.writeBlock({ off: 'M9', flood: 'M8', mist: 'M7', floodAndMist: 'M7 M8' }[c.mode] || 'M8'); },
  start() {
    if (!this.pending) return;
    this.pending = false;
    const n = this.wcs;
    const code = n >= 1 && n <= 6 ? 'G' + (53 + n) : n <= 9 ? 'G59.' + (n - 6) : null;
    if (!code) throw new Error('LinuxCNC has nine work offsets; offset ' + n + ' cannot be posted');
    post.writeBlock(gPlane('G17'), gWcs(code));
  },
  onRapid(m) { this.start(); post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), rotaryWords(m)); },
  onLinear(m) {
    this.start();
    if (m.kind === 'link' && post.property('useG0ForLinks')) { post.writeBlock(gMotion('G0'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), rotaryWords(m)); return; }
    post.writeBlock(gMotion('G1'), xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), rotaryWords(m), feedWords(m));
  },
  onRotaryPosition(r) { this.start(); post.writeBlock(gMotion('G0'), rotaryWords(r)); },
  onCircular(a) {
    this.start();
    const w = [gPlane({ xy: 'G17', xz: 'G18', yz: 'G19' }[a.plane]), gMotion(a.clockwise ? 'G2' : 'G3'), xOut(a.to.x), yOut(a.to.y), zOut(a.to.z)];
    const i = a.center.x - a.from.x, j = a.center.y - a.from.y, k = a.center.z - a.from.z;
    if (a.plane === 'xy') w.push(iOut(i), jOut(j)); else if (a.plane === 'xz') w.push(iOut(i), kOut(k)); else w.push(jOut(j), kOut(k));
    w.push(...rotaryWords(a), ...feedWords(a));
    post.writeBlock(w);
  },
  // Manual NC entries (lane C15): program stop / optional stop, and user text
  // written as a line of its own, byte for byte.
  onStop(s) { post.writeBlock(s.optional ? 'M1' : 'M0'); },
  onPassThrough(t) { post.writeln(t.text); },
  // Controller probing (lane C14's cycles): G38.2 probe move, which aborts on its own when the probe does not trip; the trigger is read
  // from #5061-#5063 (active work coordinates, length offset applied: the
  // probe tool's G43 H register must hold its calibrated sphere-bottom length),
  // and after the cycle's last point the work offset register is updated with
  // G10 L2 Pn from its stored value plus the measured displacement, the
  // arithmetic of probe::results::evaluate. Angle and inspection cycles refuse.
  onProbe(p) {
    const m = post.probeMacro(p, { dialect: 'linuxcnc', base: post.property('probeVariableBase'), eps: post.property('probeTolerance'), maxWcs: 9, endFormat: xyz, noTriggerBand: 0.00005 });
    const put = b => (typeof b === 'string' ? post.writeBlock(b) : post.writeln(b.raw));
    if (p.isFirst) post.comment('PROBE ' + p.cycle.kind + ' WCS ' + p.cycle.wcs.number + ' (' + p.cycle.pointCount + ' POINTS)');
    const e = p.overtravel, a = p.approach;
    post.writeBlock('G38.2', 'X' + xyz(e.x), 'Y' + xyz(e.y), 'Z' + xyz(e.z), 'F' + feedFmt(p.feed));
    m.point.forEach(put);
    xOut.reset(); yOut.reset(); zOut.reset(); fOut.reset(); gMotion.reset();
    if (p.isLast) {
      // Back to the approach point in the old frame, then the update.
      post.writeBlock(gMotion('G1'), xOut(a.x), yOut(a.y), zOut(a.z), fOut(p.feed));
      m.update.forEach(put);
      xOut.reset(); yOut.reset(); zOut.reset(); fOut.reset(); gMotion.reset();
    }
  },
  onDwell(d) { post.writeBlock('G4', 'P' + secFmt(d.seconds)); },
  onCycle(c) {
    this.start();
    const cy = c.cycle, r = 'R' + xyz(cy.retractZ), z = 'Z' + xyz(cy.depthZ), x = xOut(c.at.x), y = yOut(c.at.y), f = fOut(c.feed);
    let w;
    switch (cy.kind) {
      case 'drill': w = cy.dwellSeconds > 0 ? [gCycle('G82'), x, y, z, r, 'P' + secFmt(cy.dwellSeconds), f] : [gCycle('G81'), x, y, z, r, f]; break;
      case 'peck': w = [gCycle('G83'), x, y, z, r, 'Q' + xyz(cy.peck), f]; break;
      case 'chipBreak': w = [gCycle('G73'), x, y, z, r, 'Q' + xyz(cy.peck), f]; break;
      case 'tap': {
        // Rigid tapping is G33.1 (spindle-synchronized), a straight move to depth and back.
        if (!cy.rigid) post.warn('LinuxCNC floating tapping is posted as G33.1 synchronized tapping');
        gCycle.reset();
        post.writeBlock(gMotion('G0'), x, y, zOut(cy.retractZ));
        post.writeBlock('G33.1', 'Z' + xyz(cy.depthZ), 'K' + xyz(cy.pitch));
        zOut.set(cy.retractZ);
        this.inCycle = false;
        return;
      }
      case 'ream': w = [gCycle('G85'), x, y, z, r, f]; break;
      case 'bore':
        if (cy.shift > 0) throw new Error('LinuxCNC has no fine boring cycle with shift (G76); use a plain bore');
        w = cy.feedOut ? [gCycle(cy.dwellSeconds > 0 ? 'G89' : 'G85'), x, y, z, r, cy.dwellSeconds > 0 ? 'P' + secFmt(cy.dwellSeconds) : '', f] : [gCycle('G86'), x, y, z, r, cy.dwellSeconds > 0 ? 'P' + secFmt(cy.dwellSeconds) : '', f];
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
    post.writeBlock('G53', 'G0', 'Z0');
    post.writeBlock('M30');
    post.writeln('%');
  },
});
