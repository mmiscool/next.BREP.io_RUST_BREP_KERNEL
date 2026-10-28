'use strict';
// Heidenhain conversational (TNC 530/640 Klartext) post for BREP CAM.
// Written from the TNC user manual's conversational syntax; no vendor post
// code consulted or copied.

const xyz = post.length({ decimals: 3, forceSign: true });
const feedFmt = post.feed({ decimals: 0 });
const rpmFmt = post.format({ decimals: 0 });
const secFmt = post.format({ decimals: 2 });
const xOut = post.modal('X', xyz), yOut = post.modal('Y', xyz), zOut = post.modal('Z', xyz);
const fOut = post.modal('F', feedFmt);


const rotFmt = post.format({ decimals: 3, forceSign: true });
const rotOut = {};
const rotaryWords = m => !m.rotary ? [] : Object.keys(m.rotary).map(k => { if (!rotOut[k]) rotOut[k] = post.modal(k, rotFmt); return rotOut[k](m.rotary[k]); });

post.define({
  id: 'heidenhain',
  name: 'Heidenhain conversational',
  description: 'Heidenhain TNC Klartext: BEGIN/END PGM, BLK FORM, TOOL CALL, L/CC/C blocks with FMAX rapids, CYCL DEF 200/203/205/206/207/208/209 drilling and tapping, datum tables via CYCL DEF 7 (work offsets as presets).',
  vendor: 'Heidenhain',
  extension: 'h',
  units: 'auto',
  comment: t => '; ' + t,
  capabilities: {
    arcs: { planes: ['xy'], helical: false, maxSweepDeg: 360, minRadius: 0.01 },
    cycles: ['drill', 'peck', 'chipBreak', 'tap', 'ream', 'bore'],
    multiAxis: { rotary: true, workPlane: true, cycleInPlane: true, tcp: true, feedMode: 'standard' },
    linearizeTolerance: 0.01,
  },
  properties: {
    programName: { title: 'Program name override', type: 'string', default: '' },
    useFMAX: { title: 'FMAX rapids', description: 'Rapids as FMAX; otherwise F at the machine rapid', type: 'boolean', default: true },
    presetAtStart: { title: 'Activate preset (CYCL DEF 247)', description: 'Select the work offset number as a preset at every setup', type: 'boolean', default: true },
    optionalStop: { title: 'Optional stop (M1) before tool calls', type: 'boolean', default: true },
  },
  onOpen(ctx) {
    this.n = 0;
    this.count = 0;
    const name = (post.property('programName') || ctx.program.name || 'PROGRAM').toUpperCase().replace(/[^A-Z0-9_]/g, '_');
    this.name = name;
    this.units = ctx.units === 'in' ? 'INCH' : 'MM';
    this.block('BEGIN PGM ' + name + ' ' + this.units);
    if (ctx.program.comment) this.block('; ' + ctx.program.comment);
    const s = ctx.program.setups[0];
    if (s && s.stock && s.stock.kind === 'box') {
      this.block('BLK FORM 0.1 Z X' + xyz(s.stock.min[0]) + ' Y' + xyz(s.stock.min[1]) + ' Z' + xyz(s.stock.min[2]));
      this.block('BLK FORM 0.2 X' + xyz(s.stock.max[0]) + ' Y' + xyz(s.stock.max[1]) + ' Z' + xyz(s.stock.max[2]));
    }
    for (const t of ctx.program.tools) this.block('; T' + t.number + ' ' + (t.name || t.kind) + ' D' + xyz(t.geometry.diameter));
  },
  block(text) { this.n++; post.writeln(this.n + ' ' + text); },
  onSection(section) {
    this.block('; ' + section.name);
    if (section.toolChange && this.count > 0) { this.block('L Z+0 R0 FMAX M91'); if (post.property('optionalStop')) this.block('M1'); }
    this.count++;
    this.section = section;
    if (this.wcs !== section.wcs.number && post.property('presetAtStart')) { this.block('CYCL DEF 247 DATUM SETTING ~'); this.block('  Q339=' + section.wcs.number + ' ;DATUM NUMBER'); }
    this.wcs = section.wcs.number;
  },
  onToolChange(ctx) {
    const s = ctx.section;
    this.block('TOOL CALL ' + ctx.tool.number + ' Z S' + rpmFmt(s.spindleRpm));
    this.rpm = s.spindleRpm;
    xOut.reset(); yOut.reset(); zOut.reset(); fOut.reset();
    this.spindleWord = null;
  },
  onSpindle(s) {
    // The speed is in TOOL CALL; direction and stop are M codes on the next block.
    this.pendingM = s.on ? (s.direction === 'ccw' ? 'M4' : 'M3') : 'M5';
    if (s.on && s.rpm !== this.rpm) { this.block('TOOL CALL S' + rpmFmt(s.rpm)); this.rpm = s.rpm; }
    if (!s.on) { this.block('M5'); this.pendingM = null; }
  },
  onSpindleOrient(o) {
    this.block('CYCL DEF 13.0 ORIENTATION');
    this.block('CYCL DEF 13.1 ANGLE ' + post.format({ decimals: 3 })(o.angleDeg));
    this.block('M19');
  },
  onSpindleSync(s) { post.note('spindleSync ' + (s.enabled ? 'on' : 'off') + ': Heidenhain rigid tapping is CYCL DEF 207 itself; no code emitted'); },
  // Touch probe cycle 0: measure along the dominant axis of the approach
  // direction, the coordinate lands in a Q parameter.
  onCoolant(c) { this.pendingCoolant = c.mode === 'off' ? 'M9' : 'M8'; },
  flushM() { const m = [this.pendingM, this.pendingCoolant].filter(Boolean).join(' '); this.pendingM = null; this.pendingCoolant = null; return m; },
  onRapid(m) {
    const w = [xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), ...rotaryWords(m)].filter(Boolean).join(' ');
    if (!w) return;
    this.block(('L ' + w + ' R0 ' + (post.property('useFMAX') ? 'FMAX' : 'F' + feedFmt(30000)) + ' ' + this.flushM()).trim());
    fOut.reset();
  },
  onLinear(m) {
    const w = [xOut(m.to.x), yOut(m.to.y), zOut(m.to.z), ...rotaryWords(m)].filter(Boolean).join(' ');
    const f = fOut(m.feed);
    if (!w && !f) return;
    this.block(('L ' + w + ' R0 ' + f + ' ' + this.flushM()).replace(/\s+/g, ' ').trim());
  },
  onCircular(a) {
    this.block('CC X' + xyz(a.center.x) + ' Y' + xyz(a.center.y));
    const w = [xOut(a.to.x), yOut(a.to.y), zOut(a.to.z)].filter(Boolean).join(' ');
    this.block(('C ' + w + ' DR' + (a.clockwise ? '-' : '+') + ' R0 ' + fOut(a.feed) + ' ' + this.flushM()).replace(/\s+/g, ' ').trim());
  },
  // ---- multi-axis: PLANE VECTOR takes the plane's X and Z axes directly ----
  onWorkPlane(w) {
    const v = post.format({ decimals: 6, forceSign: true });
    this.block('PLANE VECTOR BX' + v(w.xAxis.x) + ' BY' + v(w.xAxis.y) + ' BZ' + v(w.xAxis.z) + ' NX' + v(w.zAxis.x) + ' NY' + v(w.zAxis.y) + ' NZ' + v(w.zAxis.z) + ' STAY');
    if (w.origin.x !== 0 || w.origin.y !== 0 || w.origin.z !== 0) this.block('; plane origin X' + xyz(w.origin.x) + ' Y' + xyz(w.origin.y) + ' Z' + xyz(w.origin.z) + ' (datum shift expected from the preset)');
    xOut.reset(); yOut.reset(); zOut.reset();
  },
  onWorkPlaneOff() { this.block('PLANE RESET STAY'); xOut.reset(); yOut.reset(); zOut.reset(); },
  onRotaryPosition(r) {
    const w = rotaryWords(r).filter(Boolean).join(' ');
    if (w) this.block('L ' + w + ' R0 FMAX');
  },
  onTcpMode(t) { this.block(t.enabled ? 'M128' : 'M129'); },
  // Manual NC entries (lane C15): M0 / M1 as blocks of their own; user text
  // untouched behind the block number every line of the program carries.
  onStop(s) { this.block(s.optional ? 'M1' : 'M0'); },
  onPassThrough(t) { this.block(t.text); },
  // Controller probing is refused by name: a TNC work offset lives in the
  // preset table, and its write (FN 17 SYSWRITE / TCH PROBE preset output) is
  // not written here without a verified convention; the cycle arithmetic is
  // the shared one and would be proven by the same oracle once it is.
  onProbe(p) { throw new Error('post "heidenhain": probe ' + p.cycle.resultVar + ': Heidenhain probing is refused: the preset-table write convention is not verified, so no work offset update is claimed'); },
  onDwell(d) { this.block('CYCL DEF 9.0 DWELL TIME'); this.block('CYCL DEF 9.1 DWELL ' + secFmt(d.seconds)); },
  onCycle(c) {
    const cy = c.cycle;
    const setup = (cy.retractZ - c.at.z) !== 0 ? Math.abs(cy.retractZ - c.at.z) : 2;
    const depth = cy.depthZ - c.at.z; // negative: below the surface at the cycle plane
    const feed = feedFmt(c.feed);
    const q = (n, v, label) => this.block('  Q' + n + '=' + v + ' ;' + label);
    switch (cy.kind) {
      case 'drill':
        this.block('CYCL DEF 200 DRILLING ~'); q(200, xyz(setup).replace('+', ''), 'SET-UP CLEARANCE'); q(201, xyz(depth), 'DEPTH'); q(206, feed, 'FEED RATE FOR PLNGNG'); q(202, xyz(Math.abs(depth)).replace('+', ''), 'PLUNGING DEPTH'); q(210, '0', 'DWELL TIME AT TOP'); q(203, xyz(c.at.z), 'SURFACE COORDINATE'); q(204, xyz(50).replace('+', ''), '2ND SET-UP CLEARANCE'); q(211, secFmt(cy.dwellSeconds || 0), 'DWELL TIME AT DEPTH'); break;
      case 'peck':
        this.block('CYCL DEF 205 UNIVERSAL PECKING ~'); q(200, xyz(setup).replace('+', ''), 'SET-UP CLEARANCE'); q(201, xyz(depth), 'DEPTH'); q(206, feed, 'FEED RATE FOR PLNGNG'); q(202, xyz(cy.peck).replace('+', ''), 'PLUNGING DEPTH'); q(203, xyz(c.at.z), 'SURFACE COORDINATE'); q(204, xyz(50).replace('+', ''), '2ND SET-UP CLEARANCE'); q(212, '0', 'DECREMENT'); q(205, xyz(cy.peck).replace('+', ''), 'MIN. PLUNGING DEPTH'); q(258, '0.2', 'UPPER ADV STOP DIST'); q(259, '0.2', 'LOWER ADV STOP DIST'); q(257, '0', 'DEPTH FOR CHIP BRKNG'); q(256, '0.2', 'DIST FOR CHIP BRKNG'); q(211, '0', 'DWELL TIME AT DEPTH'); break;
      case 'chipBreak':
        this.block('CYCL DEF 203 UNIVERSAL DRILLING ~'); q(200, xyz(setup).replace('+', ''), 'SET-UP CLEARANCE'); q(201, xyz(depth), 'DEPTH'); q(206, feed, 'FEED RATE FOR PLNGNG'); q(202, xyz(cy.peck).replace('+', ''), 'PLUNGING DEPTH'); q(210, '0', 'DWELL TIME AT TOP'); q(203, xyz(c.at.z), 'SURFACE COORDINATE'); q(204, xyz(50).replace('+', ''), '2ND SET-UP CLEARANCE'); q(212, '0', 'DECREMENT'); q(213, '0', 'NR OF CHIP BRKNG'); q(205, xyz(cy.peck).replace('+', ''), 'MIN. PLUNGING DEPTH'); q(211, '0', 'DWELL TIME AT DEPTH'); q(208, feed, 'RETRACTION FEED RATE'); q(256, xyz(cy.backOff || 0.5).replace('+', ''), 'DIST FOR CHIP BRKNG'); break;
      case 'tap':
        this.block('CYCL DEF ' + (cy.rigid ? '207 RIGID TAPPING ~' : '206 TAPPING ~')); q(200, xyz(setup).replace('+', ''), 'SET-UP CLEARANCE'); q(201, xyz(depth), 'DEPTH OF THREAD'); if (cy.rigid) q(239, xyz(cy.leftHand ? -cy.pitch : cy.pitch), 'PITCH'); else q(206, feedFmt(cy.pitch * this.section.spindleRpm), 'FEED RATE FOR PLNGNG'); if (!cy.rigid) q(211, '0', 'DWELL TIME AT DEPTH'); q(203, xyz(c.at.z), 'SURFACE COORDINATE'); q(204, xyz(50).replace('+', ''), '2ND SET-UP CLEARANCE'); break;
      case 'ream':
        this.block('CYCL DEF 201 REAMING ~'); q(200, xyz(setup).replace('+', ''), 'SET-UP CLEARANCE'); q(201, xyz(depth), 'DEPTH'); q(206, feed, 'FEED RATE FOR PLNGNG'); q(211, '0', 'DWELL TIME AT DEPTH'); q(208, feed, 'RETRACTION FEED RATE'); q(203, xyz(c.at.z), 'SURFACE COORDINATE'); q(204, xyz(50).replace('+', ''), '2ND SET-UP CLEARANCE'); break;
      case 'bore':
        this.block('CYCL DEF 202 BORING ~'); q(200, xyz(setup).replace('+', ''), 'SET-UP CLEARANCE'); q(201, xyz(depth), 'DEPTH'); q(206, feed, 'FEED RATE FOR PLNGNG'); q(211, secFmt(cy.dwellSeconds || 0), 'DWELL TIME AT DEPTH'); q(208, cy.feedOut ? feed : feedFmt(30000), 'RETRACTION FEED RATE'); q(203, xyz(c.at.z), 'SURFACE COORDINATE'); q(204, xyz(50).replace('+', ''), '2ND SET-UP CLEARANCE'); q(214, cy.shift > 0 ? '1' : '0', 'DISENGAGING DIRECTN'); q(336, '0', 'ANGLE OF SPINDLE'); break;
      default: throw new Error('unsupported cycle ' + cy.kind);
    }
    this.block('L X' + xyz(c.at.x) + ' Y' + xyz(c.at.y) + ' R0 FMAX M99');
    xOut.set(c.at.x); yOut.set(c.at.y); zOut.reset();
  },
  onComment(c) { if (!c.isSectionHeader) this.block('; ' + c.text); },
  onClose() {
    this.block('L Z+0 R0 FMAX M91');
    this.block('M30');
    this.block('END PGM ' + this.name + ' ' + this.units);
  },
});
