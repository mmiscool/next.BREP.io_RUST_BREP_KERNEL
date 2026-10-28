'use strict';
// Mach3 / Mach4 milling post for BREP CAM. Written from the Mach G-code
// references; no vendor post code consulted or copied.

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
  id: 'mach',
  name: 'Mach3 / Mach4',
  description: 'Mach3 and Mach4 milling: G54-G59 and G59 P7+ offsets (Mach4), G43 length offset, G81/G82/G83/G73/G84/G85/G86/G89 cycles, helical arcs, N numbers optional.',
  vendor: 'Artsoft',
  extension: 'tap',
  units: 'auto',
  capabilities: {
    arcs: { planes: ['xy', 'xz', 'yz'], helical: true, maxSweepDeg: 360, minRadius: 0.01 },
    cycles: ['drill', 'peck', 'chipBreak', 'tap', 'ream', 'bore'],
    multiAxis: { rotary: true, workPlane: false, tcp: false, feedMode: 'inverseTime' },
    linearizeTolerance: 0.01,
  },
  properties: {
    mach4: { title: 'Mach4 dialect', description: 'Mach4 accepts G59 Pn for offsets beyond G59', type: 'boolean', default: false },
    sequenceNumbers: { title: 'Sequence numbers', type: 'boolean', default: true },
    optionalStop: { title: 'Optional stop before tool changes', type: 'boolean', default: true },
    useG0ForLinks: { title: 'G0 for link moves', type: 'boolean', default: false },
  },
  onOpen(ctx) {
    this.count = 0;
    if (post.property('sequenceNumbers')) post.startSequence(10, 10);
    post.writeln('%');
    post.comment(ctx.program.name);
    if (ctx.program.comment) post.comment(ctx.program.comment);
    for (const t of ctx.program.tools) post.comment('T' + t.number + ' ' + (t.name || t.kind) + ' D=' + xyz(t.geometry.diameter));
    post.writeBlock('G90', 'G94', 'G17', 'G40', 'G49', 'G80', ctx.units === 'in' ? 'G20' : 'G21');
    gFeedMode.set('G94');
    gPlane.set('G17'); gCycle.set('G80');
  },
  onSection(section) {
    post.comment(section.name);
    if (section.toolChange && this.count > 0) { post.writeBlock('G53', 'G0', 'Z0'); if (post.property('optionalStop')) post.writeBlock('M1'); }
    this.count++;
    this.section = section;
    this.pending = true;
    this.wcs = section.wcs.number;
  },
  onToolChange(ctx) {
    post.writeBlock('T' + ctx.tool.number, 'M6');
    post.writeBlock('G43', 'H' + intFmt(ctx.section.lengthOffset));
    post.comment((ctx.tool.name || ctx.tool.kind) + ' D=' + xyz(ctx.tool.geometry.diameter));
    xOut.reset(); yOut.reset(); zOut.reset(); fOut.reset(); gMotion.reset(); sOut.reset();
  },
  onSpindle(s) { if (s.on) post.writeBlock(sOut(s.rpm), s.direction === 'ccw' ? 'M4' : 'M3'); else { post.writeBlock('M5'); sOut.reset(); } },
  onSpindleOrient() { throw new Error('Mach3/Mach4 have no spindle orientation code in this post; the operation cannot be posted'); },
  onSpindleSync(s) {
    if (!post.property('mach4')) throw new Error('Mach3 cannot synchronize the spindle for rigid tapping; rigid tapping needs the Mach4 dialect with a synchronizing motion controller, or a floating tap');
    post.note('spindleSync ' + (s.enabled ? 'on' : 'off') + ': Mach4 rigid tapping is the G84 cycle with a synchronizing motion controller; no code emitted');
  },
  onCoolant(c) { post.writeBlock({ off: 'M9', flood: 'M8', mist: 'M7', floodAndMist: 'M7 M8' }[c.mode] || 'M8'); },
  start() {
    if (!this.pending) return;
    this.pending = false;
    const n = this.wcs;
    let code;
    if (n >= 1 && n <= 6) code = 'G' + (53 + n);
    else if (post.property('mach4')) code = 'G59 P' + n;
    else throw new Error('Mach3 has six work offsets; offset ' + n + ' needs the Mach4 dialect');
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
        if (cy.leftHand && !post.property('mach4')) throw new Error('Mach3 has no left-hand tapping cycle; G74 needs the Mach4 dialect');
        if (cy.rigid && !post.property('mach4')) throw new Error('Mach3 cannot synchronize the spindle for rigid tapping; it needs the Mach4 dialect with a synchronizing motion controller, or a floating tap');
        w = [gCycle(cy.leftHand ? 'G74' : 'G84'), x, y, z, r, 'F' + post.feed({ decimals: 2 })(cy.pitch * this.section.spindleRpm)]; fOut.reset(); break;
      }
      case 'ream': w = [gCycle('G85'), x, y, z, r, f]; break;
      case 'bore':
        if (cy.shift > 0) throw new Error('Mach has no fine boring cycle with shift (G76)');
        w = cy.feedOut ? [gCycle(cy.dwellSeconds > 0 ? 'G89' : 'G85'), x, y, z, r, cy.dwellSeconds > 0 ? 'P' + secFmt(cy.dwellSeconds) : '', f] : [gCycle('G86'), x, y, z, r, cy.dwellSeconds > 0 ? 'P' + secFmt(cy.dwellSeconds) : '', f];
        break;
      default: throw new Error('unsupported cycle ' + cy.kind);
    }
    post.writeBlock(w);
    this.inCycle = true;
  },
  onSectionEnd() { if (this.inCycle) { post.writeBlock(gCycle('G80')); this.inCycle = false; } },
  onComment(c) { if (!c.isSectionHeader) post.comment(c.text); },
  onClose() { if (this.inCycle) post.writeBlock(gCycle('G80')); post.writeBlock('G53', 'G0', 'Z0'); post.writeBlock('M30'); post.writeln('%'); },
});
