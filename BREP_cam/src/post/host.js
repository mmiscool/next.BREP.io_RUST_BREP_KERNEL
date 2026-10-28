'use strict';
// BREP CAM post-processor host (lane C4). Evaluated before a post script on
// the shared Boa interpreter. A post calls post.define({...}) once; the host
// drives the program stream through the definition's callbacks and collects
// the text. Everything a post can reach is this object: no clock, no
// randomness, no I/O. Deterministic by construction.
const post = (() => {
  const TAU = 2 * Math.PI;
  let definition = null;
  let lines = [];
  const warnings = [];
  const notes = [];
  let properties = {};
  let unitScale = 1;               // program units -> post units
  let programUnits = 'mm';
  let postUnits = 'mm';
  let sequence = null;             // { next, increment }
  let position = { x: 0, y: 0, z: 0 };
  let positionKnown = false;
  let currentFeed = null;
  let currentSection = null;
  let spindleRpm = 0, spindleDirection = 'cw';
  let program = null;
  let toolsById = {};
  let resolved = null;             // kinematics-resolved blocks (rotary machines)
  let rotaryState = {};            // last rotary words written, name -> degrees
  let workPlaneActive = false;
  let lathe = { active: false, diameterMode: false };
  let probeCycles = [];            // the probe cycle being measured, points so far
  let probeOWord = 9000;           // LinuxCNC o-word numbers for probe checks

  const fail = message => { throw new Error(message); };
  const isFinite = v => typeof v === 'number' && Number.isFinite(v);

  // ---- formatting -------------------------------------------------------
  // post.format({decimals, forceSign, trailingZeros, scale, prefix, suffix, zero})
  // returns a function number -> text. Scaling by unit factor is the caller's
  // choice: post.length() and post.feed() are unit-aware variants.
  const makeFormat = (spec, unitAware) => {
    spec = spec || {};
    const decimals = spec.decimals === undefined ? 3 : spec.decimals;
    const forceSign = !!spec.forceSign;
    const trailingZeros = !!spec.trailingZeros;
    const scale = spec.scale === undefined ? 1 : spec.scale;
    const prefix = spec.prefix || '';
    const suffix = spec.suffix || '';
    const separator = spec.decimalSeparator || '.';
    const forceDecimal = !!spec.forceDecimal; // "10." instead of "10"
    if (!Number.isInteger(decimals) || decimals < 0 || decimals > 9) fail('format decimals must be an integer in 0..9');
    const fn = value => {
      if (!isFinite(value)) fail('cannot format a non-finite number');
      let v = value * scale * (unitAware ? unitScale : 1);
      let text = v.toFixed(decimals);
      if (!trailingZeros && decimals > 0) {
        text = text.replace(/0+$/, '');
        if (text.endsWith('.')) text = forceDecimal ? text : text.slice(0, -1);
      } else if (forceDecimal && decimals === 0) {
        text += '.';
      }
      if (/^-0\.?0*$/.test(text)) text = text.slice(1);
      if (forceSign && !text.startsWith('-')) text = '+' + text;
      if (separator !== '.') text = text.replace('.', separator);
      return prefix + text + suffix;
    };
    fn.decimals = decimals;
    return fn;
  };
  const format = spec => makeFormat(spec, false);
  const length = spec => makeFormat(spec, true);
  const feed = spec => makeFormat(spec, true);

  // post.word('X', fmt): fmt(value) -> 'X' + text, always.
  const word = (letter, fmt) => {
    if (typeof fmt !== 'function') fail('word needs a format');
    const w = v => letter + fmt(v);
    w.letter = letter;
    return w;
  };
  // post.modal('X', fmt): emits only when the formatted text changes.
  // .reset() forgets, .force() emits next time, .current() last text.
  const modal = (letter, fmt) => {
    let last = null;
    let forced = false;
    const m = v => {
      const text = fmt ? fmt(v) : String(v);
      if (!forced && last === text) return '';
      forced = false;
      last = text;
      return (letter || '') + text;
    };
    m.reset = () => { last = null; };
    m.force = () => { forced = true; };
    m.current = () => last;
    m.set = v => { last = fmt ? fmt(v) : String(v); };
    return m;
  };
  // post.group(): a modal group of codes ('G0'/'G1'/'G2'/'G3'); format(code)
  // returns the code only when it changes.
  const group = () => modal('', null);

  // ---- output -----------------------------------------------------------
  const writeln = text => { lines.push(String(text)); };
  const writeBlock = (...words) => {
    const parts = words.flat(Infinity).filter(w => w !== '' && w !== null && w !== undefined).map(String);
    if (parts.length === 0) return;
    if (sequence) {
      parts.unshift('N' + sequence.next);
      sequence.next += sequence.increment;
    }
    lines.push(parts.join(definition.wordSeparator === undefined ? ' ' : definition.wordSeparator));
  };
  const comment = text => {
    const c = definition.comment || (t => '(' + t + ')');
    writeln(c(String(text).replace(/[()]/g, '')));
  };
  const startSequence = (start, increment) => { sequence = { next: start === undefined ? 10 : start, increment: increment === undefined ? 10 : increment }; };
  const stopSequence = () => { sequence = null; };
  const warn = text => { warnings.push(String(text)); };
  const note = text => { notes.push(String(text)); };

  // ---- geometry helpers -------------------------------------------------
  const planeAxes = plane => plane === 'xy' ? ['x', 'y', 'z'] : plane === 'xz' ? ['z', 'x', 'y'] : plane === 'yz' ? ['y', 'z', 'x'] : fail('unknown arc plane ' + plane);
  const arcSweep = (from, to, center, plane, clockwise) => {
    const [u, v] = planeAxes(plane);
    const a0 = Math.atan2(from[v] - center[v], from[u] - center[u]);
    const a1 = Math.atan2(to[v] - center[v], to[u] - center[u]);
    let sweep = clockwise ? a0 - a1 : a1 - a0;
    sweep = ((sweep % TAU) + TAU) % TAU;
    if (sweep < 1e-12) sweep = TAU;
    const radius = Math.hypot(from[u] - center[u], from[v] - center[v]);
    return { sweep, radius };
  };
  const arcPoint = (arc, t) => {
    const [u, v, w] = planeAxes(arc.plane);
    const a0 = Math.atan2(arc.from[v] - arc.center[v], arc.from[u] - arc.center[u]);
    const a = a0 + (arc.clockwise ? -arc.sweep : arc.sweep) * t;
    const p = {};
    p[u] = arc.center[u] + arc.radius * Math.cos(a);
    p[v] = arc.center[v] + arc.radius * Math.sin(a);
    p[w] = arc.from[w] + arc.pitch * arc.sweep * t / TAU;
    return p;
  };
  const chordCount = (radius, sweep, tolerance) => {
    const maxStep = tolerance >= radius ? Math.PI : 2 * Math.acos(1 - tolerance / radius);
    return Math.min(3600, Math.max(1, Math.ceil(sweep / Math.max(maxStep, 1e-6))));
  };

  // ---- dispatch ---------------------------------------------------------
  const call = (name, arg, optional) => {
    const fn = definition[name];
    if (typeof fn !== 'function') {
      if (optional) return undefined;
      fail('post "' + definition.id + '" has no ' + name + ' callback');
    }
    return fn.call(definition, arg);
  };
  const point = p => ({ x: p[0], y: p[1], z: p[2] });
  const setPosition = p => { position = { x: p.x, y: p.y, z: p.z }; positionKnown = true; };
  const multiAxisCaps = () => {
    const c = (definition.capabilities || {}).multiAxis;
    return c ? { rotary: !!c.rotary, workPlane: !!c.workPlane, cycleInPlane: !!c.cycleInPlane, tcp: !!c.tcp, feedMode: c.feedMode || 'standard' } : { rotary: false, workPlane: false, cycleInPlane: false, tcp: false, feedMode: 'standard' };
  };
  // Rotary words of a block as {name: degrees} in joint order, whether they
  // changed since the last block, and the gate for posts without rotaries.
  const rotaryOf = block => {
    const words = block.rotary || [];
    const rotary = {};
    let changed = false;
    for (const w of words) {
      rotary[w.name] = w.angleDeg;
      if (rotaryState[w.name] === undefined ? Math.abs(w.angleDeg) > 1e-12 : Math.abs(rotaryState[w.name] - w.angleDeg) > 1e-12) changed = true;
    }
    if (changed && !multiAxisCaps().rotary) fail('post "' + definition.id + '" cannot move rotary axes (capabilities.multiAxis.rotary); the program needs ' + JSON.stringify(rotary));
    for (const w of words) rotaryState[w.name] = w.angleDeg;
    return { rotary: words.length ? rotary : null, rotaryChanged: changed };
  };
  const feedFields = block => ({
    feedMode: block.inverseTime !== null && block.inverseTime !== undefined ? 'inverseTime' : (block.rotaryFeed !== null && block.rotaryFeed !== undefined ? 'dpm' : 'standard'),
    inverseTime: block.inverseTime === undefined ? null : block.inverseTime,
    rotaryFeed: block.rotaryFeed === undefined ? null : block.rotaryFeed,
    toolAxis: block.motion.toolAxis ? point(block.motion.toolAxis) : null,
    repositioned: !!block.repositioned,
  });
  const linear = (block, kind, feedRate) => {
    const m = block.motion;
    const to = point(block.linear || m.to);
    const arg = { kind, from: positionKnown ? { ...position } : null, to, feed: feedRate, pathId: m.pathId === undefined ? null : m.pathId, ...rotaryOf(block), ...feedFields(block) };
    if (feedRate === null) call('onRapid', arg); else { currentFeed = feedRate; call('onLinear', arg); }
    setPosition(to);
  };
  const circular = (block, clockwise, feedRate) => {
    const m = block.motion;
    if (!positionKnown) fail('an arc has no start position');
    const to = point(block.linear || m.to), center = point(block.center || m.center);
    const from = { ...position };
    const extra = { ...rotaryOf(block), ...feedFields(block) };
    const { sweep, radius } = arcSweep(from, to, center, m.plane, clockwise);
    const arc = { kind: 'arc', clockwise, from, to, center, plane: m.plane, radius, sweep, pitch: m.pitch || 0, helical: !!m.pitch, feed: feedRate, pathId: m.pathId === undefined ? null : m.pathId, ...extra };
    const caps = definition.capabilities || {};
    const arcs = caps.arcs === undefined ? {} : caps.arcs;
    const planes = (arcs && arcs.planes) || ['xy', 'xz', 'yz'];
    const maxSweep = (arcs && arcs.maxSweepDeg !== undefined ? arcs.maxSweepDeg : 360) * Math.PI / 180;
    const helical = arcs && arcs.helical !== undefined ? arcs.helical : true;
    const minRadius = arcs && arcs.minRadius !== undefined ? arcs.minRadius : 0;
    const tolerance = caps.linearizeTolerance === undefined ? 0.01 : caps.linearizeTolerance;
    const supported = arcs !== false && planes.includes(m.plane) && (helical || !arc.helical) && radius >= minRadius;
    if (!supported) {
      // Linearize within tolerance (program units).
      const n = chordCount(radius, sweep, tolerance);
      for (let i = 1; i <= n; i++) {
        const p = arcPoint(arc, i / n);
        const q = i === n ? to : p;
        linear({ motion: { to: [q.x, q.y, q.z], pathId: m.pathId, toolAxis: m.toolAxis }, linear: null, rotary: block.rotary, inverseTime: null, rotaryFeed: null }, clockwise ? 'arcCw' : 'arcCcw', feedRate);
      }
      return;
    }
    // Split long sweeps.
    const pieces = Math.max(1, Math.ceil(sweep / maxSweep - 1e-9));
    for (let i = 1; i <= pieces; i++) {
      const end = i === pieces ? to : arcPoint(arc, i / pieces);
      const start = { ...position };
      const piece = { ...arc, from: start, to: end, sweep: sweep / pieces, center: { ...center } };
      // The centre's normal coordinate follows the start for helices.
      const [, , w] = planeAxes(m.plane);
      piece.center[w] = start[w];
      currentFeed = feedRate;
      call('onCircular', piece);
      setPosition(end);
    }
  };
  const cycleKinds = { drill: 'drill', peck: 'peck', chipBreak: 'chipBreak', tap: 'tap', ream: 'ream', bore: 'bore', backBore: 'backBore' };
  const expandCycle = (c, feedRate) => {
    const at = c.at, top = at.z, depth = c.cycle.depthZ, retract = c.cycle.retractZ;
    const rapidTo = p => { call('onRapid', { kind: 'rapid', from: { ...position }, to: p, feed: null, pathId: null }); setPosition(p); };
    const feedTo = (p, f) => { currentFeed = f; call('onLinear', { kind: 'cut', from: { ...position }, to: p, feed: f, pathId: null }); setPosition(p); };
    const dwell = s => { if (s > 0) call('onDwell', { seconds: s }); };
    rapidTo({ x: at.x, y: at.y, z: position.z });
    rapidTo({ x: at.x, y: at.y, z: top });
    const k = c.cycle.kind;
    if (k === 'drill') { feedTo({ x: at.x, y: at.y, z: depth }, feedRate); dwell(c.cycle.dwellSeconds || 0); rapidTo({ x: at.x, y: at.y, z: retract }); }
    else if (k === 'peck' || k === 'chipBreak') {
      const peck = c.cycle.peck > 0 ? c.cycle.peck : fail('peck depth must be positive');
      let z = top;
      while (z > depth + 1e-9) {
        z = Math.max(depth, z - peck);
        feedTo({ x: at.x, y: at.y, z }, feedRate);
        if (z > depth + 1e-9) {
          if (k === 'peck') { rapidTo({ x: at.x, y: at.y, z: retract }); rapidTo({ x: at.x, y: at.y, z: z + Math.min(peck, 0.5) }); }
          else { const back = c.cycle.backOff > 0 ? c.cycle.backOff : 0.5; rapidTo({ x: at.x, y: at.y, z: z + back }); }
        }
      }
      rapidTo({ x: at.x, y: at.y, z: retract });
    }
    else if (k === 'tap') {
      const sec = currentSection;
      const rpm = sec ? sec.spindleRpm : 0;
      const f = c.cycle.pitch * rpm;
      if (!(f > 0)) fail('expanded tapping needs a spindle speed and pitch');
      const dir = sec ? sec.spindleDirection : 'cw';
      const reverse = dir === 'cw' ? 'ccw' : 'cw';
      feedTo({ x: at.x, y: at.y, z: depth }, f);
      call('onSpindle', { on: true, rpm, direction: reverse });
      feedTo({ x: at.x, y: at.y, z: retract }, f);
      call('onSpindle', { on: true, rpm, direction: dir });
    }
    else if (k === 'ream') { feedTo({ x: at.x, y: at.y, z: depth }, feedRate); feedTo({ x: at.x, y: at.y, z: retract }, feedRate); }
    else if (k === 'bore') {
      feedTo({ x: at.x, y: at.y, z: depth }, feedRate); dwell(c.cycle.dwellSeconds || 0);
      if (c.cycle.feedOut) feedTo({ x: at.x, y: at.y, z: retract }, feedRate); else rapidTo({ x: at.x, y: at.y, z: retract });
    }
    else fail('cycle "' + k + '" cannot be expanded; the post must support it');
  };
  const cycle = (block, feedRate) => {
    const m = block.motion;
    if (workPlaneActive) {
      // Lane C10's convention (multiaxis::indexed): inside an indexed section
      // the cycle's depths run along the plane's Z and are plane-frame values;
      // the resolver gives `at` in plane-local coordinates. So the cycle posts
      // as an ordinary cycle in the rotated plane (G68.2 + G81.., PLANE + CYCL
      // DEF 200), on controls that run cycles in a rotated plane.
      if (!multiAxisCaps().cycleInPlane) fail('post "' + definition.id + '" refuses a canned ' + m.cycle.kind + ' cycle inside a tilted work plane: no cycle-in-plane output (capabilities.multiAxis.cycleInPlane)');
      if (!block.linear) fail('a canned ' + m.cycle.kind + ' cycle inside a tilted work plane needs the kinematics-resolved stream (plane-local coordinates); give the program its rotary machine');
    }
    const at = point(block.linear || m.at);
    const c = { kind: 'cycle', at, cycle: { ...m.cycle }, feed: feedRate };
    const caps = definition.capabilities || {};
    const supported = Array.isArray(caps.cycles) && caps.cycles.includes(m.cycle.kind) && typeof definition.onCycle === 'function';
    if (supported) { currentFeed = feedRate; call('onCycle', c); setPosition({ x: at.x, y: at.y, z: at.z }); }
    else expandCycle(c, feedRate);
  };

  // Surface speed: program units/min -> m/min (metric post) or ft/min (inch post).
  const surfaceSpeed = v => postUnits === 'in' ? v * unitScale / 12 : v * unitScale / 1000;
  const threadCut = (block, feedRate) => {
    const m = block.motion;
    if (!positionKnown) fail('a thread cut has no start position');
    const to = point(block.linear || m.to);
    const arg = { kind: 'threadCut', from: { ...position }, to, pitch: m.pitch, feed: feedRate, pathId: m.pathId === undefined ? null : m.pathId, ...rotaryOf(block), ...feedFields(block) };
    currentFeed = null;
    call('onThreadCut', arg);
    setPosition(to);
  };
  // A canned turning thread: the post takes the explicit schedule when it
  // claims `turnThreadCycle`; otherwise one synchronized pass per depth,
  // exactly as turning::thread_cycle::expand emits them.
  const turnThread = (block, feedRate) => {
    const c = block.motion.cycle;
    const caps = definition.capabilities || {};
    if (caps.turnThreadCycle && typeof definition.onTurnThreadCycle === 'function') {
      call('onTurnThreadCycle', { cycle: c, feed: feedRate, ...rotaryOf(block) });
      setPosition({ x: c.clearanceX, y: 0, z: c.start[2] });
      return;
    }
    const tool = toolsById[currentSection.tool];
    const plunge = tool && tool.feeds ? tool.feeds.plungeFeed : null;
    if (!(plunge > 0)) fail('expanding a turning thread needs the tool plunge feed');
    if (!(feedRate > 0)) fail('a turning thread needs its synchronized feed (pitch x rpm)');
    const sign = c.internal ? 1 : -1;
    const total = c.depths.length + (c.springPasses || 0);
    const at = (x, z) => ({ motion: { to: [x, 0, z] }, linear: null, rotary: block.rotary, inverseTime: null, rotaryFeed: null });
    linear(at(c.clearanceX, c.start[2]), 'rapid', null);
    for (let i = 0; i < total; i++) {
      const idx = Math.min(i, c.depths.length - 1);
      const x = c.start[0] + sign * c.depths[idx];
      const shift = c.infeedOffsets[idx];
      linear({ ...at(c.clearanceX, c.start[2] + shift), motion: { to: [c.clearanceX, 0, c.start[2] + shift], pathId: i } }, 'rapid', null);
      linear({ ...at(x, c.start[2] + shift), motion: { to: [x, 0, c.start[2] + shift], pathId: i } }, 'plunge', plunge);
      threadCut({ motion: { to: [x, 0, c.end[2] + shift], pitch: c.pitch, pathId: i }, linear: null, rotary: block.rotary, inverseTime: null, rotaryFeed: null }, feedRate);
      linear({ ...at(c.clearanceX, c.end[2] + shift), motion: { to: [c.clearanceX, 0, c.end[2] + shift], pathId: i } }, 'retract', null);
    }
    linear(at(c.clearanceX, c.start[2]), 'rapid', null);
  };

  const run = input => {
    if (!definition) fail('the post script did not call post.define');
    program = input.program;
    properties = resolveProperties(input.properties || {});
    programUnits = program.units || 'mm';
    postUnits = definition.units === 'auto' || definition.units === undefined ? programUnits : definition.units;
    unitScale = programUnits === postUnits ? 1 : (postUnits === 'in' ? 1 / 25.4 : 25.4);
    lines = []; position = { x: 0, y: 0, z: 0 }; positionKnown = false; currentFeed = null; currentSection = null;
    spindleRpm = 0; spindleDirection = 'cw';
    toolsById = {};
    for (const t of program.tools) toolsById[t.id] = t;
    resolved = input.resolved || null;
    rotaryState = {}; workPlaneActive = false; lathe = { active: false, diameterMode: false }; probeCycles = []; probeOWord = 9000;
    if (resolved && resolved.joints && resolved.joints.length && !multiAxisCaps().rotary) {
      // Rotary machines still post fine when nothing turns; the gate is per block.
      note('program resolved for rotary axes ' + resolved.joints.join('') + '; this post writes no rotary words, so every block must keep them at zero');
    }
    call('onOpen', { program, properties, units: postUnits, programUnits, fileName: input.fileName || program.name, joints: resolved ? resolved.joints : [] });
    // Blocks of one section: the resolver's (coordinates and rotary words per
    // block, inserted blocks included) or the program's own motions.
    const blocksOf = section => {
      if (resolved) {
        const [first, count] = resolved.sections[section.index];
        return resolved.motions.slice(first, first + count);
      }
      return program.motions.slice(section.firstMotion, section.firstMotion + section.motionCount).map((pm, k) => ({
        source: section.firstMotion + k, motion: pm.motion, linear: null, center: null, rotary: [], inverseTime: null, rotaryFeed: null, repositioned: false,
      }));
    };
    // Live-tool milling on a mill-turn (integrator 2026-10-20, mill-turn post
    // safety): a section that interpolates in polar or cylindrical
    // coordinates, indexes C, or moves C with the linear axes, runs with the main
    // spindle stopped, the C axis engaged as a rotary axis and the live tool
    // spinning, and the control returns to turning afterwards. The post owns
    // the codes (onLiveTooling); the host owns the order: the section's
    // spindle start becomes the transition into live tooling (so its M3 can
    // never start the main spindle), and the section's end the transition
    // back. A post without onLiveTooling is refused by name.
    const millTurn = !!(program.machine && program.machine.kind && program.machine.kind.kind === 'millTurn');
    const LIVE_KINDS = ['polarInterpolation', 'cylindricalInterpolation', 'linearRotary', 'rotaryPosition'];
    const SETUP_KINDS = ['comment', 'toolChange', 'coolant', 'spindleOn', 'spindleOff'];
    for (const section of program.sections) {
      currentSection = section;
      const tool = toolsById[section.tool];
      const blocks = blocksOf(section);
      const liveKinds = millTurn ? [...new Set(blocks.map(b => b.motion.kind).filter(k => LIVE_KINDS.includes(k)))] : [];
      if (liveKinds.length && typeof definition.onLiveTooling !== 'function') {
        fail('post "' + definition.id + '" cannot post live-tool milling on a mill-turn: section "' + section.name + '" uses ' + liveKinds.join(', ') + ', which needs the control taken out of turning (main spindle stop, C axis engaged, live spindle, XY plane) and back again, and this post has no onLiveTooling transition (the haas-lathe and fanuc-lathe posts have one)');
      }
      let liveOn = false;
      const liveEnter = (rpm, direction) => {
        if (liveOn) return;
        liveOn = true;
        spindleRpm = rpm; spindleDirection = direction;
        call('onLiveTooling', { enabled: true, rpm, direction, interpolation: liveKinds, section });
      };
      call('onSection', { ...section, toolDefinition: tool, isFirst: section.index === 0, isLast: section.index === program.sections.length - 1 });
      for (const block of blocks) {
        const m = block.motion;
        if (liveKinds.length && !liveOn && !SETUP_KINDS.includes(m.kind)) liveEnter(section.spindleRpm, section.spindleDirection);
        if (liveKinds.length && (m.kind === 'spindleOn' || m.kind === 'spindleOff')) {
          // The live spindle starts with the transition and stops with the way back.
          if (m.kind === 'spindleOn') liveEnter(m.rpm, m.direction);
          continue;
        }
        // Feeds stay in program units; post.feed() formatters convert.
        const feedRate = m.feed === 'rapid' || m.feed === undefined ? null : m.feed;
        switch (m.kind) {
          case 'rapid': linear(block, 'rapid', null); break;
          case 'plunge': case 'cut': case 'leadIn': case 'leadOut': case 'link': case 'retract':
            linear(block, m.kind, feedRate); break;
          case 'arcCw': circular(block, true, feedRate); break;
          case 'arcCcw': circular(block, false, feedRate); break;
          case 'dwell': call('onDwell', { seconds: m.seconds }); break;
          case 'toolChange': call('onToolChange', { tool: toolsById[m.tool], section }); break;
          case 'cycle': cycle(block, feedRate); break;
          case 'spindleOn': spindleRpm = m.rpm; spindleDirection = m.direction; call('onSpindle', { on: true, rpm: m.rpm, direction: m.direction }); break;
          case 'spindleOff': call('onSpindle', { on: false, rpm: 0, direction: section.spindleDirection }); break;
          // M19-class orientation and M29-class synchronization (rigid tapping):
          // a post implements each or throws by name; the host never drops them.
          case 'spindleOrient': call('onSpindleOrient', { angleDeg: m.angleDeg }); break;
          case 'spindleSync': call('onSpindleSync', { enabled: m.enabled, rpm: spindleRpm, direction: spindleDirection }); break;
          case 'coolant': call('onCoolant', { mode: m.mode }); break;
          // ---- manual NC entries (lane C15) ----
          case 'stop': call('onStop', { optional: !!m.optional }); break;
          // The text reaches the post byte for byte; the post writes it as a
          // line of its own (no word formatting, no sequence number).
          case 'passThrough': call('onPassThrough', { text: String(m.text) }); break;
          case 'comment': {
            // The comment assembly puts first in a section is the section name.
            const header = block === blocks[0] && m.text === section.name;
            if (typeof definition.onComment === 'function') call('onComment', { text: m.text, isSectionHeader: header }); else comment(m.text);
            break;
          }
          // ---- multi-axis (lane C10's resolver supplies the words) ----
          case 'workPlane': {
            if (!multiAxisCaps().workPlane) fail('post "' + definition.id + '" has no tilted work plane output (capabilities.multiAxis.workPlane); run the operation without a work plane or choose another post');
            workPlaneActive = true;
            const r = rotaryOf(block);
            call('onWorkPlane', { origin: point(m.origin), xAxis: point(m.xAxis), zAxis: point(m.zAxis), ...r });
            positionKnown = false;
            break;
          }
          case 'workPlaneOff': {
            if (!multiAxisCaps().workPlane) fail('post "' + definition.id + '" has no tilted work plane output (capabilities.multiAxis.workPlane)');
            workPlaneActive = false;
            call('onWorkPlaneOff', { ...rotaryOf(block) });
            positionKnown = false;
            break;
          }
          case 'rotaryPosition': {
            const r = rotaryOf({ ...block, rotary: block.rotary && block.rotary.length ? block.rotary : m.axes });
            if (!multiAxisCaps().rotary) fail('post "' + definition.id + '" cannot position rotary axes (capabilities.multiAxis.rotary)');
            call('onRotaryPosition', { axes: m.axes, ...r });
            break;
          }
          case 'tcpMode': {
            if (m.enabled && !multiAxisCaps().tcp) fail('post "' + definition.id + '" has no tool-centre-point control (capabilities.multiAxis.tcp)');
            call('onTcpMode', { enabled: m.enabled, ...rotaryOf(block) });
            break;
          }
          // ---- turning (lane C11's typed motions) ----
          case 'latheMode': {
            lathe = { active: true, diameterMode: !!m.diameterMode };
            call('onLatheMode', { diameterMode: lathe.diameterMode });
            break;
          }
          case 'spindleSurfaceSpeed': {
            spindleRpm = m.maxRpm; spindleDirection = m.direction;
            call('onSpindleSurfaceSpeed', { surfaceSpeed: surfaceSpeed(m.surfaceSpeed), surfaceSpeedProgramUnits: m.surfaceSpeed, maxRpm: m.maxRpm, direction: m.direction });
            break;
          }
          case 'threadCut': threadCut(block, feedRate); break;
          case 'turnThreadCycle': turnThread(block, feedRate); break;
          // ---- live tooling (lane C11b) and probing (lane C14) ----
          case 'linearRotary': {
            // The motion carries its own rotary words; they override the resolver's.
            linear({ ...block, rotary: m.axes, inverseTime: null, rotaryFeed: null }, 'linearRotary', feedRate);
            break;
          }
          case 'polarInterpolation': {
            call('onPolarInterpolation', { enabled: !!m.enabled });
            positionKnown = false;
            break;
          }
          case 'cylindricalInterpolation': {
            call('onCylindricalInterpolation', { radius: m.radius === undefined ? null : m.radius, enabled: m.radius !== null && m.radius !== undefined });
            positionKnown = false;
            break;
          }
          case 'probe': {
            // Lane C14's contract: a post that does not define onProbe refuses
            // probe motions by name; one that does gets each point with the
            // cycle so far, and the complete cycle at its last point (the only
            // place a work offset may change).
            if (typeof definition.onProbe !== 'function') fail('post "' + definition.id + '" does not post probe motions: the probing contract (lane C14) refuses them unless the post implements controller probing (the Fanuc, Haas, LinuxCNC, Mazak, Okuma and Siemens 840D posts do); a probe-specific post must define onProbe');
            const c = m.cycle;
            const approach = point(c.approach), target = point(c.target);
            const d = { x: target.x - approach.x, y: target.y - approach.y, z: target.z - approach.z };
            const len = Math.hypot(d.x, d.y, d.z);
            if (!(len > 0)) fail('a probe move needs distinct approach and target points');
            const dir = { x: d.x / len, y: d.y / len, z: d.z / len };
            const overtravel = { x: target.x + dir.x * c.overtravel, y: target.y + dir.y * c.overtravel, z: target.z + dir.z * c.overtravel };
            if (c.pointIndex === 0) probeCycles = [];
            const first = probeCycles[0];
            if (c.pointIndex !== probeCycles.length || (first && (first.kind !== c.kind || first.pointCount !== c.pointCount || JSON.stringify(first.wcs) !== JSON.stringify(c.wcs)))) {
              fail('probe ' + c.resultVar + ': point ' + (c.pointIndex + 1) + ' of ' + c.pointCount + ' does not continue the ' + (first ? first.kind : 'open') + ' cycle (points of one cycle must be consecutive)');
            }
            probeCycles.push(c);
            const isLast = c.pointIndex === c.pointCount - 1;
            call('onProbe', { cycle: c, cycles: probeCycles.slice(), isFirst: c.pointIndex === 0, isLast, approach, target, direction: dir, overtravel, feed: c.feed, spindleRpm, from: positionKnown ? { ...position } : null, ...rotaryOf(block) });
            if (isLast) probeCycles = [];
            // Where the stylus stopped is unknown until the control reports it.
            positionKnown = false;
            break;
          }
          default: fail('unknown motion kind ' + m.kind);
        }
      }
      if (probeCycles.length) fail('probe cycle ' + probeCycles[0].kind + ' ends after ' + probeCycles.length + ' of ' + probeCycles[0].pointCount + ' points: the work offset is updated only after every point');
      if (liveOn) {
        call('onLiveTooling', { enabled: false, rpm: 0, direction: spindleDirection, interpolation: liveKinds, section });
        positionKnown = false;
      }
      call('onSectionEnd', section, true);
    }
    currentSection = null;
    call('onClose', { program });
    const text = lines.join(definition.lineSeparator === undefined ? '\n' : definition.lineSeparator) + (definition.lineSeparator === undefined ? '\n' : definition.lineSeparator);
    return { text, extension: definition.extension || 'nc', warnings, notes, properties };
  };

  // ---- properties -------------------------------------------------------
  const propertySpecs = () => {
    const out = {};
    const specs = definition.properties || {};
    for (const name of Object.keys(specs)) {
      const s = specs[name];
      if (!s || typeof s !== 'object') fail('property ' + name + ' must be an object');
      const type = s.type || (typeof s.default === 'boolean' ? 'boolean' : typeof s.default === 'number' ? 'number' : 'string');
      out[name] = { title: s.title || name, description: s.description || '', type, default: s.default, values: s.values || null, min: s.min === undefined ? null : s.min, max: s.max === undefined ? null : s.max };
    }
    return out;
  };
  const coerce = (name, spec, value) => {
    if (value === undefined || value === null) return spec.default;
    switch (spec.type) {
      case 'boolean':
        if (typeof value === 'boolean') return value;
        if (value === 'true' || value === '1') return true;
        if (value === 'false' || value === '0') return false;
        fail('property ' + name + ' must be a boolean');
      case 'number': case 'integer': {
        const n = typeof value === 'number' ? value : Number(value);
        if (!Number.isFinite(n)) fail('property ' + name + ' must be a number');
        if (spec.type === 'integer' && !Number.isInteger(n)) fail('property ' + name + ' must be an integer');
        if (spec.min !== null && n < spec.min) fail('property ' + name + ' is below its minimum ' + spec.min);
        if (spec.max !== null && n > spec.max) fail('property ' + name + ' is above its maximum ' + spec.max);
        return n;
      }
      case 'enum':
        if (!spec.values.includes(value)) fail('property ' + name + ' must be one of ' + spec.values.join(', '));
        return value;
      default:
        return String(value);
    }
  };
  const resolveProperties = given => {
    const specs = propertySpecs();
    const out = {};
    for (const name of Object.keys(given)) if (!specs[name]) fail('unknown post property ' + name);
    for (const name of Object.keys(specs)) out[name] = coerce(name, specs[name], given[name]);
    return Object.freeze(out);
  };

  // ---- controller probing (lane C14's arithmetic, probe::results::evaluate) ----
  // Macro text for one probe point and, at the cycle's last point, the work
  // offset update, in one of these dialects:
  //   fanuc     Fanuc / Haas / Mazak EIA macro B: #n, [ ], IF [..] THEN #3000,
  //             skip position #5061-#5063, G10 L2 Pn from #5221 + 20 (n - 1) + axis
  //   linuxcnc  #n, [ ], oN if / (abort, ..) / oN endif, G38.2 result #5061-#5063, G10 L2
  //   okuma     OSP user task: VCn, [ ], IF [..] Nlabel, VUACM/VDOUT[992] alarm,
  //             skip position VSKPX/Y/Z, work offset VZOFX/Y/Z[n] += displacement
  //   siemens   SINUMERIK: Rn, ( ), IF .. SETAL ENDIF, $AC_MEA[1] trigger flag,
  //             $AA_MW[axis] measured value, $P_UIFR[n,axis,TR] += displacement
  // Triggers are read in the active work coordinates with the length offset
  // applied; the update adds the measured displacement to the register's
  // stored value, so the new origin is O_old + delta exactly as evaluate
  // defines it.
  const PROBE_KINDS = ['singleSurface', 'outsideCorner', 'insideCorner', 'web', 'pocket', 'rectangle', 'bore', 'boss'];
  const PROBE_DIALECTS = {
    fanuc: {
      v: k => '#' + k, L: '[', R: ']', trig: ['#5061', '#5062', '#5063'], noTrigger: 'position',
      op: { GT: 'GT', LT: 'LT', LE: 'LE' },
      alarm: (cond, code, message) => ['IF [' + cond + '] THEN #3000=' + code + '(' + message + ')'],
      update: (n, words) => ['G10 L2 P' + n + ' ' + words.map(([axis, e]) => 'XYZ'[axis] + '[#' + (5221 + 20 * (n - 1) + axis) + '+' + e + ']').join(' ')],
    },
    linuxcnc: {
      v: k => '#' + k, L: '[', R: ']', trig: ['#5061', '#5062', '#5063'], noTrigger: 'none',
      op: { GT: 'GT', LT: 'LT', LE: 'LE' },
      alarm: (cond, code, message, k) => [{ raw: 'o' + k + ' if [' + cond + ']' }, { raw: '(abort, ' + message + ')' }, { raw: 'o' + k + ' endif' }],
      update: (n, words) => ['G10 L2 P' + n + ' ' + words.map(([axis, e]) => 'XYZ'[axis] + '[#' + (5221 + 20 * (n - 1) + axis) + '+' + e + ']').join(' ')],
    },
    okuma: {
      v: k => 'VC' + k, L: '[', R: ']', trig: ['VSKPX', 'VSKPY', 'VSKPZ'], noTrigger: 'position',
      // OSP branches on a condition: jump over the alarm when it holds the
      // other way (every probe condition is one comparison).
      op: { GT: 'GT', LT: 'LT', LE: 'LE', GE: 'GE' }, inverse: { GT: 'LE', LT: 'GE', LE: 'GT' },
      alarm: (cond, code, message, k) => ['IF [' + cond + '] NPR' + k, "VUACM[1]='" + message.slice(0, 16) + "'", 'VDOUT[992]=' + code, { raw: 'NPR' + k }],
      update: (n, words) => words.map(([axis, e]) => { const r = 'VZOF' + 'XYZ'[axis] + '[' + n + ']'; return r + '=' + r + '+' + e; }),
    },
    siemens: {
      v: k => 'R' + k, L: '(', R: ')', trig: ['$AA_MW[X]', '$AA_MW[Y]', '$AA_MW[Z]'], noTrigger: 'flag',
      op: { GT: '>', LT: '<', LE: '<=' },
      alarm: (cond, code, message) => ['IF ' + cond + ' ; ' + message, 'SETAL(' + (65000 + code) + ')', 'ENDIF'],
      update: (n, words) => words.map(([axis, e]) => { const r = '$P_UIFR[' + n + ',' + 'XYZ'[axis] + ',TR]'; return r + '=' + r + '+' + e; }),
    },
  };
  const probeMacro = (p, o) => {
    const c = p.cycle;
    const dl = PROBE_DIALECTS[o.dialect];
    if (!dl) fail('probe macro dialect ' + o.dialect + ' is not known');
    const who = 'post "' + definition.id + '": probe ' + c.resultVar;
    if (c.kind === 'inspection' || !c.wcs) fail(who + ': inspection probing records measurements, which this post cannot store or report; only work-offset cycles post');
    if (c.kind === 'angle') fail(who + ': an angle cycle measures a rotation, which a work offset translation cannot hold; refused');
    if (PROBE_KINDS.indexOf(c.kind) < 0) fail(who + ': probe cycle kind ' + c.kind + ' is not implemented');
    const n = c.wcs.number;
    if (!(Number.isInteger(n) && n >= 1 && n <= o.maxWcs)) fail(who + ': work offset ' + n + ' has no writable register on this control (1-' + o.maxWcs + ')');
    if (p.spindleRpm > 0) fail(who + ': the spindle runs at S' + p.spindleRpm + ' during probing; give the probe tool a spindle speed of 0');
    if (p.rotary && Object.keys(p.rotary).length) fail(who + ': probing with rotary words is not implemented');
    const base = o.base, eps = o.eps;
    const num = x => { let t = (Math.abs(x) < 5e-13 ? 0 : x).toFixed(6).replace(/0+$/, '').replace(/\.$/, ''); if (t === '-0') t = '0'; return t; };
    const L = x => num(x * unitScale);
    const br = x => dl.L + x + dl.R;
    const fn = (name, x) => name + dl.L + x + dl.R;
    // a - constant, without a doubled sign (a - -2 is written a+2).
    const sub = (a, k) => (k.charAt(0) === '-' ? a + '+' + k.slice(1) : a + '-' + k);
    const v = k => dl.v(base + k);
    const X = i => v(3 * i), Y = i => v(3 * i + 1), Z = i => v(3 * i + 2);
    const out = [];
    const alarm = (lhs, op, rhs, code, message) => {
      const shown = dl.inverse ? dl.inverse[op] : op;
      out.push(...dl.alarm(lhs + ' ' + dl.op[shown] + ' ' + rhs, code, message, probeOWord++));
    };
    const i = c.pointIndex;
    if (dl.noTrigger === 'position') {
      // A skip move that reached its end point did not trigger.
      const e = p.overtravel, f = o.endFormat;
      alarm(br(fn('ABS', sub(dl.trig[0], num(+f(e.x)))) + '+' + fn('ABS', sub(dl.trig[1], num(+f(e.y)))) + '+' + fn('ABS', sub(dl.trig[2], num(+f(e.z))))), 'LT', num(o.noTriggerBand), 1, 'PROBE ' + c.nominal.id.toUpperCase() + ' DID NOT TRIGGER');
    } else if (dl.noTrigger === 'flag') {
      out.push(...dl.alarm('$AC_MEA[1]==0', 1, 'PROBE ' + c.nominal.id.toUpperCase() + ' DID NOT TRIGGER', probeOWord++));
    }
    out.push(X(i) + '=' + dl.trig[0], Y(i) + '=' + dl.trig[1], Z(i) + '=' + dl.trig[2]);
    const pointBlocks = out.splice(0, out.length);
    if (!p.isLast) return { point: pointBlocks, update: [] };
    const cs = p.cycles;
    const terms = list => list.filter(([k]) => k !== 0).map(([k, e]) => num(k) + '*' + e).join('+').replace(/\+-/g, '-');
    const delta = [null, null, null];
    const sq = (a, b) => br(a + '-' + b) + '*' + br(a + '-' + b);
    if (c.kind === 'bore' || c.kind === 'boss') {
      if (cs.some(q => Math.abs(q.stylusRadius - cs[0].stylusRadius) > eps)) fail(who + ': a circle cycle needs one stylus radius');
      const s = 12, AX = v(s), AY = v(s + 1), BX = v(s + 2), BY = v(s + 3), D = v(s + 4), U = v(s + 5), V = v(s + 6), CX = v(s + 7), CY = v(s + 8), RR = v(s + 9);
      for (let k = 1; k < 4; k++) alarm(fn('ABS', Z(k) + '-' + Z(0)), 'GT', num(eps), 2, 'PROBE CIRCLE CONTACTS NOT AT ONE Z');
      out.push(AX + '=' + br(X(1) + '-' + X(0)), AY + '=' + br(Y(1) + '-' + Y(0)), BX + '=' + br(X(2) + '-' + X(0)), BY + '=' + br(Y(2) + '-' + Y(0)));
      out.push(D + '=' + br(AX + '*' + BY + '-' + AY + '*' + BX));
      alarm(fn('ABS', D), 'LE', br(num(eps) + '*' + br(fn('SQRT', AX + '*' + AX + '+' + AY + '*' + AY) + '+' + fn('SQRT', BX + '*' + BX + '+' + BY + '*' + BY))), 3, 'PROBE CIRCLE CONTACTS DEGENERATE');
      out.push(U + '=' + br(br(AX + '*' + AX + '+' + AY + '*' + AY) + '*0.5'), V + '=' + br(br(BX + '*' + BX + '+' + BY + '*' + BY) + '*0.5'));
      out.push(CX + '=' + br(X(0) + '+' + br(U + '*' + BY + '-' + V + '*' + AY) + '/' + D), CY + '=' + br(Y(0) + '+' + br(AX + '*' + V + '-' + BX + '*' + U) + '/' + D));
      out.push(RR + '=' + fn('SQRT', sq(CX, X(0)) + '+' + sq(CY, Y(0))));
      for (let k = 1; k < 4; k++) alarm(fn('ABS', fn('SQRT', sq(CX, X(k)) + '+' + sq(CY, Y(k))) + '-' + RR), 'GT', num(eps), 4, 'PROBE CONTACTS NOT ON ONE CIRCLE');
      if (c.kind === 'boss') alarm(br(sub(RR, L(cs[0].stylusRadius))), 'LE', num(eps), 5, 'PROBE BOSS RADIUS INVALID');
      const nom = [0, 1].map(k => (cs[0].nominal.position[k] + cs[1].nominal.position[k]) * 0.5);
      delta[0] = sub(CX, L(nom[0]));
      delta[1] = sub(CY, L(nom[1]));
    } else {
      // e_i = (t_i - target_i) . n_i: the measured surface's normal offset.
      const e = cs.map((q, k) => {
        const name = v(12 + k), t = q.target, nn = q.nominal.normal;
        const parts = [[nn[0], br(sub(X(k), L(t[0])))], [nn[1], br(sub(Y(k), L(t[1])))], [nn[2], br(sub(Z(k), L(t[2])))]];
        out.push(name + '=' + br(terms(parts)));
        return name;
      });
      const nrm = k => cs[k].nominal.normal;
      const coef = {
        singleSurface: () => [[nrm(0), 1, 0]],
        outsideCorner: () => [[nrm(0), 1, 0], [nrm(1), 1, 1]],
        insideCorner: () => [[nrm(0), 1, 0], [nrm(1), 1, 1]],
        web: () => [[nrm(0), 0.5, 0], [nrm(0), -0.5, 1]],
        pocket: () => [[nrm(0), 0.5, 0], [nrm(0), -0.5, 1]],
        rectangle: () => [[nrm(0), 0.5, 0], [nrm(0), -0.5, 1], [nrm(2), 0.5, 2], [nrm(2), -0.5, 3]],
      }[c.kind]();
      for (let axis = 0; axis < 3; axis++) {
        const t = terms(coef.map(([nn, k, j]) => [nn[axis] * k, e[j]]));
        if (t) delta[axis] = t;
      }
    }
    const words = [];
    delta.forEach((d, axis) => {
      if (d === null) return;
      const name = v(26 + axis);
      out.push(name + '=' + br(d));
      words.push([axis, name]);
    });
    if (!words.length) fail(who + ': the ' + c.kind + ' cycle observes no axis');
    out.push(...dl.update(n, words));
    return { point: pointBlocks, update: out };
  };

  const describe = () => {
    if (!definition) fail('the post script did not call post.define');
    return { id: definition.id, name: definition.name, description: definition.description || '', vendor: definition.vendor || '', extension: definition.extension || 'nc', units: definition.units || 'auto', machine: definition.machine || 'mill', capabilities: definition.capabilities || {}, properties: propertySpecs() };
  };

  const define = d => {
    if (definition) fail('post.define was called twice');
    if (!d || typeof d !== 'object') fail('post.define needs an object');
    if (typeof d.id !== 'string' || !d.id || typeof d.name !== 'string' || !d.name) fail('a post needs an id and a name');
    definition = d;
  };

  return Object.freeze({
    define, format, length, feed, word, modal, group,
    writeln, writeBlock, comment, warn, note, startSequence, stopSequence,
    property: name => { if (!(name in properties)) fail('unknown post property ' + name); return properties[name]; },
    units: () => postUnits, programUnits: () => programUnits, toUnits: v => v * unitScale,
    position: () => ({ ...position }), feedRate: () => currentFeed, section: () => currentSection, program: () => program,
    lathe: () => ({ ...lathe }), surfaceSpeed, probeMacro, rotaryJoints: () => (resolved ? resolved.joints.slice() : []),
    // ZXZ Euler angles (degrees) of the frame with the given unit X and Z
    // axes: R = Rz(i) Rx(j) Rz(k) maps frame coordinates to setup coordinates,
    // the convention of Fanuc/Haas/Mazak G68.2 I J K.
    eulerZXZ: (xAxis, zAxis) => {
      const z = [zAxis.x, zAxis.y, zAxis.z], x = [xAxis.x, xAxis.y, xAxis.z];
      const y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
      const deg = a => a * 180 / Math.PI;
      const c = Math.max(-1, Math.min(1, z[2]));
      const j = Math.acos(c);
      if (Math.sin(j) < 1e-9) {
        // Degenerate: Z along +-Z; one rotation about Z carries the X axis.
        return { i: deg(Math.atan2(x[1], x[0])) * (c < 0 ? -1 : 1), j: deg(j), k: 0 };
      }
      return { i: deg(Math.atan2(z[0], -z[1])), j: deg(j), k: deg(Math.atan2(x[2], y[2])) };
    },
    arcSweep, arcPoint,
    wcs: number => number >= 1 && number <= 6 ? 'G' + (53 + number) : null,
    __run: run, __describe: describe,
  });
})();
// No ambient time or randomness in a post.
globalThis.Date = undefined; globalThis.performance = undefined;
Math.random = () => { throw new Error('ambient randomness unsupported'); };
function __brepRunPost(input) { return post.__run(input); }
function __brepDescribePost() { return post.__describe(); }
