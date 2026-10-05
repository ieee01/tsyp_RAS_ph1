'use strict';
// LivingMap command post. Everything arrives as gateway snapshots over /ws;
// commands go back through /api/demo (the gateway uplink). Offline: no CDN.

const $ = id => document.getElementById(id);
// Two views of one page: the read-only command post, and the simulation harness.
const HARNESS = location.pathname.replace(/\/+$/, '') === '/harness';
document.body.classList.add(HARNESS ? 'view-harness' : 'view-command');
const canvas = $('map');
const ctx = canvas.getContext('2d');

const COLORS = {
  writer: '#56d4e8', executor: '#f2b84b', victim: '#5fd99a', gas: '#f0727f',
  fire: '#ff9a4d', beacon: '#8fb3c4', gateway: '#b28cff', bad: '#f0727f', warn: '#f2b84b',
  junction: '#ffd34d', relay: '#c3a6ff', mesh: '#4fc3d9', firebot: '#ff5c5c',
  grid: 'rgba(120, 160, 180, .07)', gridMajor: 'rgba(120, 160, 180, .14)',
  free: '#11222c', occupied: '#9bb8c4', text: '#e6f0f4', muted: '#8ba3b1',
};
const EVENT = {
  0: { key: 'nav', name: 'Navigation memory', short: 'NAV', color: COLORS.beacon, icon: 'i-beacon' },
  1: { key: 'victim', name: 'Victim', short: 'V01', color: COLORS.victim, icon: 'i-victim' },
  2: { key: 'gas', name: 'Gas hazard', short: 'H01', color: COLORS.gas, icon: 'i-gas' },
  3: { key: 'blocked', name: 'Blocked path', short: 'P01', color: '#c9a0ff', icon: 'i-beacon' },
  4: { key: 'fire', name: 'Fire', short: 'F01', color: COLORS.fire, icon: 'i-fire' },
};
const KEEPOUT_M = { 2: 2.0, 3: 0.7, 4: 1.8 };   // Executor hard keep-out per hazard type
const GATEWAY = [-1.5, 0];         // map frame
// Reference mine walls in the Writer map frame (world - (-11, 0)).
const WALLS = [[-3, 5, 21, 5], [-3, -5, 21, -5], [21, -5, 21, 5], [3, -1, 3, 5], [5, 1, 13, 1],
  [11.25, -1, 14.75, -1], [11, 1, 11, 5], [16, -1, 16, 5], [13.5, -3, 18.5, -3]];
const PHASES = {
  READY: 'Ready when you are', STARTING: 'Starting exploration', EXPLORING: 'Writer exploring',
  PAUSED: 'Writer paused', FAILING_WRITER: 'Preserving knowledge', WAITING_FAILURE: 'Confirming failure',
  STICKING_WRITER: 'Immobilising the Writer', WAITING_STUCK: 'Confirming STUCK',
  MEMORY_PRESERVED: 'Knowledge handed over', SECURING: 'Securing the scene', EXECUTING: 'Rescue under way',
  COMPLETE: 'Mission complete',
  ERROR: 'Sequence needs attention',
};
const SPEEDS = { normal: 'Normal speed', fast: 'Fast', very_fast: 'Very fast' };

let state = { memories: [], robots: {}, mission: {}, network: {} };
let connected = false, submitting = false, restarting = false;
let speedSelected = false, modeSelected = false;
let occupancy = null, mapKey = '';
let view = null;                    // world bounds currently shown
let hover = null;                   // {x, y} canvas px
let memFilter = 'all';
let scenario = null;
const placing = () => HARNESS && Boolean($('placement-type').value);
const layers = { trails: true, mesh: true, links: false, zones: true, walls: false };
const HANDOVER = ['FAILED', 'STUCK', 'HOME'];
const ROBOT_NAMES = { writer: 'Writer', executor: 'Executor', firebot: 'Fire robot' };
const RESCUE = ['executor', 'firebot'];
const tx = m => (m.target_x ?? m.x), ty = m => (m.target_y ?? m.y);
const resolved = m => (m.roles || []).includes('RESOLVED');
const bid = id => `B${String(id).padStart(3, '0')}`;
const outcome = () => document.querySelector('input[name=outcome]:checked')?.value || 'destroyed';

const escapeHTML = value => String(value ?? '').replace(/[&<>"']/g,
  ch => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch]));
const num = (v, n = 2) => Number.isFinite(v) ? v.toFixed(n) : '—';
const fresh = () => connected && state.received_age_s != null && state.received_age_s < 5;
const eventInfo = type => EVENT[type] || EVENT[0];

/* ---------- Map geometry ---------- */
function resize() {
  const ratio = window.devicePixelRatio || 1;
  canvas.width = canvas.clientWidth * ratio;
  canvas.height = canvas.clientHeight * ratio;
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
}
new ResizeObserver(resize).observe(canvas);

function contentBounds() {
  const b = { xmin: -3.5, xmax: 21.5, ymin: -5.5, ymax: 5.5 };   // mine extent
  if (placing()) return b;
  const grow = (x, y) => {
    if (!Number.isFinite(x) || !Number.isFinite(y)) return;
    b.xmin = Math.min(b.xmin, x); b.xmax = Math.max(b.xmax, x);
    b.ymin = Math.min(b.ymin, y); b.ymax = Math.max(b.ymax, y);
  };
  const m = state.map;
  if (m) { grow(m.origin_x, m.origin_y); grow(m.origin_x + m.width * m.resolution, m.origin_y + m.height * m.resolution); }
  for (const r of Object.values(state.robots || {})) grow(r.x, r.y);
  for (const mem of state.memories || []) { grow(mem.x, mem.y); grow(tx(mem), ty(mem)); }
  return b;
}
function updateView() {
  const target = contentBounds();
  if (!view) { view = { ...target }; return; }
  // Only ever expand (so the map does not jitter), easing toward the target.
  for (const k of ['xmin', 'ymin']) view[k] += (Math.min(view[k], target[k]) - view[k]) * 0.2;
  for (const k of ['xmax', 'ymax']) view[k] += (Math.max(view[k], target[k]) - view[k]) * 0.2;
}
function scale() {
  const pad = 36;
  return Math.min((canvas.clientWidth - pad * 2) / (view.xmax - view.xmin),
                  (canvas.clientHeight - pad * 2 - 40) / (view.ymax - view.ymin));
}
function pt(x, y) {
  const s = scale();
  const ox = (canvas.clientWidth - (view.xmax - view.xmin) * s) / 2;
  const oy = (canvas.clientHeight - 40 - (view.ymax - view.ymin) * s) / 2;
  return [ox + (x - view.xmin) * s, oy + (view.ymax - y) * s];
}

function cacheMap() {
  const m = state.map;
  if (!m) return;
  const key = `${m.width}x${m.height}@${m.origin_x},${m.origin_y}:${m.runs?.length}:${JSON.stringify(m.runs?.slice(0, 40))}`;
  if (key === mapKey) return;
  mapKey = key;
  occupancy = document.createElement('canvas');
  occupancy.width = m.width; occupancy.height = m.height;
  const image = occupancy.getContext('2d').createImageData(m.width, m.height);
  const free = [17, 34, 44, 255], occ = [155, 184, 196, 255], unknown = [0, 0, 0, 0];
  let cell = 0;
  for (const [value, count] of m.runs) {
    for (let j = 0; j < count && cell < m.width * m.height; j++, cell++) {
      const row = Math.floor(cell / m.width), col = cell % m.width;
      image.data.set(value > 20 ? occ : value >= 0 ? free : unknown, ((m.height - 1 - row) * m.width + col) * 4);
    }
  }
  occupancy.getContext('2d').putImageData(image, 0, 0);
}

/* ---------- Map drawing ---------- */
function drawGrid() {
  const s = scale();
  for (let x = Math.ceil(view.xmin); x <= view.xmax; x++) {
    ctx.strokeStyle = x % 5 === 0 ? COLORS.gridMajor : COLORS.grid;
    ctx.beginPath(); ctx.moveTo(...pt(x, view.ymin)); ctx.lineTo(...pt(x, view.ymax)); ctx.stroke();
  }
  for (let y = Math.ceil(view.ymin); y <= view.ymax; y++) {
    ctx.strokeStyle = y % 5 === 0 ? COLORS.gridMajor : COLORS.grid;
    ctx.beginPath(); ctx.moveTo(...pt(view.xmin, y)); ctx.lineTo(...pt(view.xmax, y)); ctx.stroke();
  }
  // Scale bar (bottom right, above the legend line).
  const metres = s > 40 ? 2 : 5;
  const [x0, y0] = [canvas.clientWidth - 24 - metres * s, canvas.clientHeight - 22];
  ctx.strokeStyle = COLORS.muted; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(x0, y0 - 4); ctx.lineTo(x0, y0); ctx.lineTo(x0 + metres * s, y0); ctx.lineTo(x0 + metres * s, y0 - 4); ctx.stroke();
  ctx.lineWidth = 1;
  ctx.fillStyle = COLORS.muted; ctx.font = '11px ui-monospace, monospace'; ctx.textAlign = 'center';
  ctx.fillText(`${metres} m`, x0 + metres * s / 2, y0 - 7);
  ctx.textAlign = 'left';
}

function drawZone(m, info) {
  const [x, y] = pt(tx(m), ty(m)), r = (KEEPOUT_M[m.event_type] || 1.5) * scale();
  const g = ctx.createRadialGradient(x, y, 0, x, y, r);
  g.addColorStop(0, info.color + '55'); g.addColorStop(0.7, info.color + '1f'); g.addColorStop(1, info.color + '00');
  ctx.fillStyle = g; ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fill();
  ctx.setLineDash([4, 5]); ctx.strokeStyle = info.color + '80';
  ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
}

function beaconPoint(m) {
  const roles = m.roles || [];
  const [x, y] = pt(m.x, m.y);
  const color = roles.includes('JUNCTION') ? COLORS.junction : roles.includes('RELAY') ? COLORS.relay
    : roles.includes('ENTRANCE') ? COLORS.victim : m.event_type ? eventInfo(m.event_type).color : COLORS.beacon;
  ctx.fillStyle = color; ctx.strokeStyle = color;
  if (roles.includes('JUNCTION')) {
    ctx.beginPath(); ctx.moveTo(x, y - 5); ctx.lineTo(x + 5, y); ctx.lineTo(x, y + 5); ctx.lineTo(x - 5, y); ctx.closePath(); ctx.fill();
  } else if (roles.includes('ENTRANCE')) {
    ctx.fillRect(x - 4, y - 4, 8, 8);
  } else {
    ctx.beginPath(); ctx.arc(x, y, 3, 0, Math.PI * 2); ctx.fill();
  }
  if (roles.includes('RELAY')) { ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(x, y, 7, 0, Math.PI * 2); ctx.stroke(); ctx.lineWidth = 1; }
  else if (!m.event_type) { ctx.globalAlpha *= .6; ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2); ctx.stroke(); ctx.globalAlpha /= .6; }
  if (m.destroyed) {
    ctx.strokeStyle = COLORS.bad; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(x - 6, y - 6); ctx.lineTo(x + 6, y + 6); ctx.moveTo(x + 6, y - 6); ctx.lineTo(x - 6, y + 6); ctx.stroke();
    ctx.lineWidth = 1;
  }
}

function drawMarker(m) {
  const info = eventInfo(m.event_type);
  ctx.globalAlpha = m.freshness === 'EXPIRED' ? .35 : 1;
  beaconPoint(m);
  if (!m.event_type) {
    if (m.executor_confirmed) {
      const [bx, by] = pt(m.x, m.y);
      ctx.strokeStyle = COLORS.executor; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(bx, by, 9, 0, Math.PI * 2); ctx.stroke(); ctx.lineWidth = 1;
    }
    ctx.globalAlpha = 1;
    return;
  }
  // Event: the beacon sits at a safe standoff and points at the event.
  const [bx, by] = pt(m.x, m.y), [x, y] = pt(tx(m), ty(m));
  if (resolved(m)) ctx.globalAlpha = .55;
  if (Math.hypot(x - bx, y - by) > 2) {
    ctx.strokeStyle = info.color + 'aa'; ctx.setLineDash([3, 4]); ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(x, y); ctx.stroke(); ctx.setLineDash([]); ctx.lineWidth = 1;
  }
  ctx.fillStyle = info.color; ctx.strokeStyle = info.color;
  if (m.event_type === 1) {            // victim: cross
    ctx.shadowColor = info.color; ctx.shadowBlur = 12;
    const a = 7, b = 2.6;
    ctx.fillRect(x - b, y - a, b * 2, a * 2); ctx.fillRect(x - a, y - b, a * 2, b * 2);
  } else if (m.event_type === 2) {     // gas: dot + ring
    ctx.shadowColor = info.color; ctx.shadowBlur = 12;
    ctx.beginPath(); ctx.arc(x, y, 4.5, 0, Math.PI * 2); ctx.fill();
    ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, 9, 0, Math.PI * 2); ctx.stroke(); ctx.lineWidth = 1;
  } else if (m.event_type === 4) {     // fire: flame
    ctx.shadowColor = info.color; ctx.shadowBlur = 12;
    ctx.beginPath(); ctx.moveTo(x, y - 10);
    ctx.bezierCurveTo(x + 9, y - 2, x + 7, y + 8, x, y + 8);
    ctx.bezierCurveTo(x - 7, y + 8, x - 9, y - 2, x, y - 10); ctx.fill();
  } else {
    ctx.fillRect(x - 5, y - 5, 10, 10);
  }
  ctx.shadowBlur = 0;
  if (m.executor_confirmed) {
    ctx.globalAlpha = 1; ctx.strokeStyle = COLORS.executor; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(bx, by, 9, 0, Math.PI * 2); ctx.stroke(); ctx.lineWidth = 1;
  }
  ctx.globalAlpha = 1;
  if (resolved(m)) {                   // dealt with by a rescue robot
    ctx.strokeStyle = COLORS.victim; ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.moveTo(x - 6, y + 1); ctx.lineTo(x - 1, y + 6); ctx.lineTo(x + 8, y - 6); ctx.stroke(); ctx.lineWidth = 1;
  }
  const done = resolved(m) ? (m.event_type === 4 ? ' · OUT' : m.event_type === 1 ? ' · ASSISTED' : ' · RESOLVED') : '';
  label(`${info.name.toUpperCase()} · ${bid(m.beacon_id)}${done}`, x + 14, y - 10, info.color);
}

function drawMesh(memories) {
  const mesh = state.mesh || {};
  const pos = id => id === 'gateway' ? GATEWAY : (mesh.beacons || {})[id];
  for (const [id, route] of Object.entries(mesh.routes || {})) {
    const a = pos(id), b = pos(String(route.next));
    if (!a || !b) continue;
    const p = route.p ?? 1;
    ctx.strokeStyle = p >= .9 ? COLORS.mesh + 'cc' : p >= .6 ? COLORS.warn + 'cc' : COLORS.bad + 'cc';
    ctx.lineWidth = 1.6;
    ctx.beginPath(); ctx.moveTo(...pt(...a)); ctx.lineTo(...pt(...b)); ctx.stroke();
  }
  for (const id of mesh.unreachable || []) {
    const a = pos(String(id)); if (!a) continue;
    const [x, y] = pt(...a);
    ctx.strokeStyle = COLORS.bad; ctx.setLineDash([2, 3]);
    ctx.beginPath(); ctx.arc(x, y, 10, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
  }
  // Each robot's first radio hop into the mesh.
  for (const [id, r] of Object.entries(state.robots || {})) {
    const route = r.route;
    if (!route || !Number.isFinite(r.x) || r.comms === 'SILENT') continue;
    const first = route.connected ? (route.path?.length ? pos(String(route.path[0])) : GATEWAY) : null;
    if (!first) continue;
    ctx.strokeStyle = (COLORS[id] || COLORS.text) + 'aa'; ctx.setLineDash([5, 4]); ctx.lineWidth = 1.4;
    ctx.beginPath(); ctx.moveTo(...pt(r.x, r.y)); ctx.lineTo(...pt(...first)); ctx.stroke();
    ctx.setLineDash([]); ctx.lineWidth = 1;
  }
  ctx.lineWidth = 1;
}

function label(text, x, y, color) {
  ctx.font = '600 11px system-ui, sans-serif';
  const w = ctx.measureText(text).width + 10;
  ctx.fillStyle = 'rgba(7, 13, 18, .82)';
  ctx.beginPath(); ctx.roundRect(x - 5, y - 11, w, 16, 4); ctx.fill();
  ctx.fillStyle = color; ctx.fillText(text, x, y + 1);
}

function heading(r) {
  const h = r.history || [];
  for (let i = h.length - 1; i > 0; i--) {
    const dx = h[i][0] - h[i - 1][0], dy = h[i][1] - h[i - 1][1];
    if (Math.hypot(dx, dy) > 0.05) return Math.atan2(dy, dx);
  }
  return 0;
}

function drawRobot(id, r) {
  if (!Number.isFinite(r.x)) return;
  const [x, y] = pt(r.x, r.y);
  const failed = r.state === 'FAILED' || r.state === 'MISSION_FAILED';
  const stuck = r.state === 'STUCK';
  const color = failed ? COLORS.bad : stuck ? COLORS.warn : COLORS[id] || COLORS.text;
  const t = performance.now() / 1000;
  if (!failed) {                                   // pulsing halo
    const pulse = 14 + 6 * ((t * 0.8) % 1);
    ctx.strokeStyle = color + '55'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, y, pulse, 0, Math.PI * 2); ctx.stroke(); ctx.lineWidth = 1;
  }
  ctx.save(); ctx.translate(x, y); ctx.rotate(-heading(r));
  ctx.fillStyle = color; ctx.shadowColor = color; ctx.shadowBlur = 14;
  ctx.beginPath(); ctx.moveTo(11, 0); ctx.lineTo(-8, 8); ctx.lineTo(-4, 0); ctx.lineTo(-8, -8); ctx.closePath(); ctx.fill();
  ctx.restore(); ctx.shadowBlur = 0;
  if (failed) {
    ctx.strokeStyle = '#ffd5da'; ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.moveTo(x - 6, y - 6); ctx.lineTo(x + 6, y + 6); ctx.moveTo(x + 6, y - 6); ctx.lineTo(x - 6, y + 6); ctx.stroke();
    ctx.lineWidth = 1;
  }
  const tag = failed ? ' · FAILED' : stuck ? ' · STUCK' : r.state === 'HOME' ? ' · HOME' : '';
  label(`${(ROBOT_NAMES[id] || id).toUpperCase()}${tag}${r.comms === 'SILENT' ? ' · RADIO SILENT' : ''}`, x + 14, y + 20, color);
}

function draw() {
  ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  updateView();
  drawGrid();
  const m = state.map;
  if (occupancy && m && !placing()) {
    const [x, y] = pt(m.origin_x, m.origin_y + m.height * m.resolution);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(occupancy, x, y, m.width * m.resolution * scale(), m.height * m.resolution * scale());
  }
  if (layers.walls || placing()) {
    ctx.strokeStyle = '#b39a74'; ctx.lineWidth = 3; ctx.setLineDash([7, 5]);
    for (const w of WALLS) { ctx.beginPath(); ctx.moveTo(...pt(w[0], w[1])); ctx.lineTo(...pt(w[2], w[3])); ctx.stroke(); }
    ctx.setLineDash([]); ctx.lineWidth = 1;
  }
  const memories = placing() ? [] : state.memories || [];
  if (layers.zones) for (const mem of memories) if ((mem.event_type === 2 || mem.event_type === 4) && !resolved(mem)) drawZone(mem, eventInfo(mem.event_type));

  if (layers.trails && !placing()) {
    for (const [id, r] of Object.entries(state.robots || {})) {
      const h = r.history || [];
      if (h.length < 2) continue;
      ctx.strokeStyle = (COLORS[id] || COLORS.text) + '99'; ctx.lineWidth = 2.5; ctx.lineJoin = 'round';
      ctx.beginPath(); h.forEach((p, i) => i ? ctx.lineTo(...pt(...p)) : ctx.moveTo(...pt(...p))); ctx.stroke();
      ctx.lineWidth = 1;
    }
  }
  if (layers.mesh && !placing()) drawMesh(memories);
  if (layers.links) {
    const byId = new Map(memories.map(mem => [mem.beacon_id, mem]));
    ctx.strokeStyle = 'rgba(143, 179, 196, .45)'; ctx.setLineDash([2, 5]);
    for (const mem of memories) {
      const prev = byId.get(mem.previous_beacon_id);
      if (prev) { ctx.beginPath(); ctx.moveTo(...pt(prev.x, prev.y)); ctx.lineTo(...pt(mem.x, mem.y)); ctx.stroke(); }
    }
    ctx.setLineDash([]);
  }
  // Gateway
  const [gx, gy] = pt(...GATEWAY);
  ctx.fillStyle = COLORS.gateway; ctx.shadowColor = COLORS.gateway; ctx.shadowBlur = 12;
  ctx.beginPath(); ctx.moveTo(gx, gy - 8); ctx.lineTo(gx + 7, gy); ctx.lineTo(gx, gy + 8); ctx.lineTo(gx - 7, gy); ctx.closePath(); ctx.fill();
  ctx.shadowBlur = 0; label('GATEWAY', gx - 30, gy - 16, COLORS.gateway);

  for (const mem of [...memories].sort((a, b) => a.event_type - b.event_type)) drawMarker(mem);
  if (!placing()) for (const [id, r] of Object.entries(state.robots || {})) drawRobot(id, r);
  if (placing() && scenario) {
    for (const event of scenario.events) {
      const [x, y] = pt(event.x, event.y);
      const info = Object.values(EVENT).find(e => e.key ===
        ({ VICTIM: 'victim', GAS_HAZARD: 'gas', FIRE: 'fire' })[event.type]);
      ctx.strokeStyle = info.color;
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(x, y, 9, 0, Math.PI * 2); ctx.stroke();
      ctx.lineWidth = 1;
      label(`${event.id} · NEXT RUN`, x + 14, y - 10, info.color);
    }
  }
  updateTooltip();
  requestAnimationFrame(draw);
}

/* ---------- Tooltip ---------- */
function updateTooltip() {
  const tip = $('tooltip');
  if (!hover || !view || placing()) { tip.hidden = true; return; }
  let best = null, bestD = 16;
  for (const mem of state.memories || []) {
    const [x, y] = pt(mem.x, mem.y), d = Math.hypot(x - hover.x, y - hover.y);
    if (d < bestD) { best = { kind: 'mem', item: mem }; bestD = d; }
  }
  for (const [id, r] of Object.entries(state.robots || {})) {
    const [x, y] = pt(r.x, r.y), d = Math.hypot(x - hover.x, y - hover.y);
    if (d < bestD) { best = { kind: 'robot', id, item: r }; bestD = d; }
  }
  if (!best) { tip.hidden = true; return; }
  const key = best.kind + (best.id || best.item.beacon_id) + Math.floor(performance.now() / 500);
  if (tip.dataset.key !== key) {
    tip.dataset.key = key;
    if (best.kind === 'mem') {
      const m = best.item, info = eventInfo(m.event_type);
      const g = m.guidance;
      tip.innerHTML = `<b style="color:${info.color}">${bid(m.beacon_id)} · ${escapeHTML(info.name)}</b>${(m.roles || []).length ? ` · ${escapeHTML(m.roles.join(' / '))}` : ''}<br>
        ${escapeHTML(m.freshness)} · ${Math.floor(m.age_s || 0)} s old · TTL ${m.ttl_sec} s${m.hops ? ` · ${m.hops} radio hop${m.hops > 1 ? 's' : ''}` : ''}<br>
        Confidence ${num(m.confidence * 100, 0)}%${m.severity ? ` · severity ${m.severity}` : ''}<br>
        ${g ? `${turnText(m) ? `${escapeHTML(turnText(m))}, then g` : 'G'}o ${num(g.range_m, 1)} m ${escapeHTML(g.compass)} (${num(g.compass_deg, 0)}°) to ${escapeHTML(g.to)}<br>` : ''}
        <span class="mono">beacon ${num(m.x)}, ${num(m.y)}${m.target_x != null ? `<br>event ${num(m.target_x)}, ${num(m.target_y)}` : ''}<br>GPS ${num(m.gps?.lat, 6)}, ${num(m.gps?.lon, 6)}</span>
        ${m.destroyed ? '<br><span style="color:#f0727f">✕ Beacon destroyed · memory kept by the gateway</span>' : ''}
        ${m.executor_confirmed ? '<br><span style="color:#f2b84b">✓ Heard by the Executor</span>' : ''}`;
    } else {
      const r = best.item;
      tip.innerHTML = `<b style="color:${COLORS[best.id]}">${best.id.toUpperCase()}</b> · ${escapeHTML(r.state || '—')}<br>
        ${escapeHTML(r.detail || '')}<br>${escapeHTML(routeText(r))}`;
    }
  }
  const wrap = canvas.getBoundingClientRect();
  tip.hidden = false;
  const left = Math.min(hover.x + 16, wrap.width - tip.offsetWidth - 8);
  const top = hover.y + 16 + tip.offsetHeight > wrap.height ? hover.y - tip.offsetHeight - 12 : hover.y + 16;
  tip.style.left = `${Math.max(8, left)}px`; tip.style.top = `${Math.max(8, top)}px`;
}
canvas.addEventListener('mousemove', e => { const r = canvas.getBoundingClientRect(); hover = { x: e.clientX - r.left, y: e.clientY - r.top }; });
canvas.addEventListener('mouseleave', () => { hover = null; });

/* Next-run setup uses reference Writer odometry, independent of SLAM drift. */
function showPlacement() {
  canvas.classList.toggle('placing', placing());
}

function setPlacement(x, y) {
  const event = scenario?.events.find(e => e.id === $('placement-type').value);
  if (!event) return;
  x = Math.round(x * 100) / 100;
  y = Math.round(y * 100) / 100;
  const blocked = scenario.walls.some(w => Math.abs(x - w.x) <= w.width / 2 + 0.4 && Math.abs(y - w.y) <= w.height / 2 + 0.4);
  if (!Number.isFinite(x) || !Number.isFinite(y) || x < -2.5 || x > 20.5 || y < -4.5 || y > 4.5 || blocked) {
    $('placement-status').textContent = 'Choose a point inside the mine, at least 0.4 m from walls.';
    return;
  }
  event.x = x;
  event.y = y;
  $('placement-status').textContent = `${event.id} selected. Restart simulation to apply.`;
  showPlacement();
}

$('placement-type').addEventListener('change', () => {
  view = contentBounds();
  showPlacement();
  $('placement-status').textContent = placing() ? 'Click the mine map to place this event. Changes apply after restart.' : 'Showing live discoveries.';
});
canvas.addEventListener('click', e => {
  if (!placing() || !view) return;
  const rect = canvas.getBoundingClientRect();
  const [originX, originY] = pt(0, 0);
  setPlacement((e.clientX - rect.left - originX) / scale(), (originY - (e.clientY - rect.top)) / scale());
});

async function loadScenario() {
  try {
    const response = await fetch('/api/scenario');
    if (!response.ok) throw new Error('Scenario setup unavailable');
    scenario = await response.json();
    showPlacement();
    $('placement-status').textContent = 'Choose an event, then click the mine map to place it.';
  } catch (error) {
    $('placement-status').textContent = error.message;
  }
}
showPlacement();
if (HARNESS) loadScenario();

/* ---------- Panels ---------- */
function stateClass(s) {
  if (!s) return '';
  if (['FAILED', 'MISSION_FAILED'].includes(s)) return 'bad';
  if (s === 'MISSION_COMPLETE') return 'ok';
  if (['ONLINE', 'EXPLORING', 'NAVIGATING', 'RETURNING', 'ASSISTING', 'EXTINGUISHING', 'REPORTING'].includes(s)) return 'busy';
  if (s === 'HOME') return 'ok';
  if (['PAUSED', 'MISSION_READY', 'STUCK'].includes(s)) return 'warn';
  return '';
}
function setText(id, text) { const el = $(id); if (el.textContent !== text) el.textContent = text; }

function renderRobot(id, r) {
  const s = $(`${id}-state`);
  s.textContent = r.state || 'WAITING';
  s.className = `state ${stateClass(r.state)}`;
  setText(`${id}-detail`, r.detail || 'Waiting for telemetry');
  const age = state.generated_at && r.last_seen ? state.generated_at - r.last_seen : null;
  const ageEl = $(`${id}-age`);
  ageEl.textContent = age == null ? '—' : `${num(age, 1)} s ago${r.comms === 'SILENT' ? ' · radio silent' : ''}`;
  ageEl.className = `mono ${age == null ? '' : age < 3 ? 'ok-text' : age < 10 ? 'warn-text' : 'bad-text'}`;
  ageEl.title = routeText(r);
  const battery = Number.isFinite(r.battery) ? r.battery : null;
  setText(`${id}-battery`, battery == null ? '—' : `${num(battery, 0)} %`);
  const bar = $(`${id}-battery-bar`);
  bar.style.width = `${battery ?? 0}%`;
  bar.style.background = battery == null ? 'transparent' : battery > 40 ? 'var(--ok)' : battery > 20 ? 'var(--warn)' : 'var(--bad)';
}

function routeText(r) {
  const route = r.route;
  if (!route) return 'Radio route unknown';
  if (!route.connected) return `No radio route out (best link ${num((route.uplink_p || 0) * 100, 0)}%)`;
  if (!route.path?.length) return `Direct to gateway · ${num(route.p * 100, 0)}% delivery`;
  return `Via ${route.path.map(bid).join(' → ')} · ${route.hops} hops · ${num(route.p * 100, 0)}% delivery`;
}

function renderNetwork() {
  const n = state.network || {};
  const link = n.long_distance_link || 'WAITING';
  setText('net-gateway', n.gateway || '—');
  $('net-gateway').className = n.gateway === 'ONLINE' ? 'ok-text' : 'warn-text';
  setText('net-link', link);
  $('net-link').className = link === 'ONLINE' ? 'ok-text' : link === 'WAITING' ? 'warn-text' : 'bad-text';
  setText('net-rssi', Number.isFinite(n.last_rssi_dbm) ? `${num(n.last_rssi_dbm, 0)} dBm` : '—');
  const p = n.last_delivery_probability;
  setText('net-prob', Number.isFinite(p) ? `${num(p * 100, 0)}%` : '—');
  const bar = $('net-prob-bar');
  bar.style.width = `${Number.isFinite(p) ? p * 100 : 0}%`;
  bar.style.background = p >= .8 ? 'var(--ok)' : p >= .4 ? 'var(--warn)' : 'var(--bad)';
  setText('net-rx', String(n.packets_received || 0));
  setText('net-drop', String(n.packets_dropped || 0));
  setText('net-queue', String(n.queued_snapshots || 0));
  const mesh = state.mesh || {};
  const dead = new Set(mesh.dead || []);
  const onAir = Object.keys(mesh.beacons || {}).filter(id => !dead.has(Number(id))).length;
  const hops = Object.values(mesh.routes || {}).map(route => route.hops || 0);
  setText('mesh-alive', String(onAir));
  setText('mesh-hops', hops.length ? String(Math.max(...hops)) : '—');
  setText('mesh-cut', String((mesh.unreachable || []).length + dead.size));
  for (const id of ['writer', 'executor']) {
    const r = state.robots?.[id] || {}, route = r.route;
    setText(`mesh-${id}`, r.comms === 'SILENT' ? 'radio silent' : route ? (route.connected
      ? `${route.hops} hop${route.hops > 1 ? 's' : ''} · ${num(route.p * 100, 0)}%` : 'out of reach') : '—');
  }
  setText('net', `Gateway ${fresh() ? 'live' : 'stale'} · link ${link} · ${n.stale_snapshots_discarded || 0} superseded snapshots discarded during outages`);
  setText('link-text', `Link ${link === 'ONLINE' ? 'online' : link.toLowerCase()}`);
  $('link-led').className = `led ${link === 'ONLINE' ? 'live' : link === 'WAITING' ? '' : 'down'}`;
}

// "Turn right" is relative to how you arrive, so it is derived here from the
// previous beacon's guidance (the direction of arrival), not stored in the frame.
function turnText(m) {
  const prev = (state.memories || []).find(p => p.beacon_id === m.previous_beacon_id);
  const into = prev?.guidance?.bearing_deg, out = m.guidance?.bearing_deg;
  if (!Number.isFinite(into) || !Number.isFinite(out) || prev.next_beacon_id !== m.beacon_id) return '';
  const turn = ((out - into + 540) % 360) - 180;          // counter-clockwise positive
  if (Math.abs(turn) < 30) return 'straight on';
  if (Math.abs(turn) > 150) return 'turn back';
  return turn > 0 ? (turn > 100 ? 'sharp left' : 'turn left') : (turn < -100 ? 'sharp right' : 'turn right');
}

function guidanceText(m) {
  const g = m.guidance;
  if (!g) return '';
  const arrow = `<span class="arrow" style="transform:rotate(${(g.compass_deg || 0) - 90}deg)">➜</span>`;
  const turn = turnText(m);
  const place = (m.roles || []).find(r => ['CORRIDOR', 'JUNCTION', 'DEAD END'].includes(r));
  return `<div class="guidance">${arrow} ${turn ? `${turn} · ` : ''}${num(g.range_m, 1)} m ${escapeHTML(g.compass)} to ${escapeHTML(g.to)}${place ? ` · ${escapeHTML(place.toLowerCase())}` : ''}</div>`;
}

function memoryCard(m) {
  const info = eventInfo(m.event_type);
  const roles = [...(m.roles || []), ...(m.destroyed ? ['DESTROYED'] : [])];
  const ttl = Math.max(1, m.ttl_sec || 1), lifeLeft = Math.max(0, 1 - (m.age_s || 0) / ttl);
  const lifeColor = m.freshness === 'FRESH' ? 'var(--ok)' : m.freshness === 'EXPIRED' ? 'var(--bad)' : 'var(--warn)';
  return `<article class="memory ${m.freshness === 'EXPIRED' ? 'expired' : ''} ${m.destroyed ? 'destroyed' : ''}" style="--accent:${info.color}">
    <div class="mem-head"><svg class="ico"><use href="#${info.icon}"/></svg><strong>B${String(m.beacon_id).padStart(3, '0')}</strong>
      <span class="mem-name">${escapeHTML(info.name)}</span>
      <span class="fresh-tag f-${escapeHTML(m.freshness)}">${escapeHTML(m.freshness)}</span></div>
    <div class="mem-bars">
      <div><label><span>Lifetime left</span><span class="mono">${Math.floor(m.age_s || 0)} / ${ttl} s</span></label><div class="bar"><span style="width:${lifeLeft * 100}%;background:${lifeColor}"></span></div></div>
      <div><label><span>Confidence</span><span class="mono">${num(m.confidence * 100, 0)}%</span></label><div class="bar"><span style="width:${(m.confidence || 0) * 100}%;background:${info.color}"></span></div></div>
    </div>
    ${roles.length ? `<div class="role-tags">${roles.map(r => `<span class="role-tag r-${escapeHTML(r.replace(' ', '-'))}">${escapeHTML(r)}</span>`).join('')}</div>` : ''}
    ${guidanceText(m)}
    <div class="mem-coords">${m.target_x != null ? `EVENT ${num(m.target_x)}, ${num(m.target_y)} · ` : ''}BEACON ${num(m.x)}, ${num(m.y)}<br>
      GPS ${num(m.gps?.lat, 6)}, ${num(m.gps?.lon, 6)}<br>
      way out ${m.previous_beacon_id ? bid(m.previous_beacon_id) : 'entrance'} · way in ${m.next_beacon_id ? bid(m.next_beacon_id) : 'end'} · seq ${m.sequence ?? '—'}${m.hops ? ` · ${m.hops} hop${m.hops > 1 ? 's' : ''}` : ''}</div>
    ${m.executor_confirmed ? '<span class="heard"><svg class="ico"><use href="#i-check"/></svg>Heard by the Executor</span>' : ''}
  </article>`;
}

let lastMemHTML = '', lastTimelineHTML = '';
function render() {
  cacheMap();
  const w = state.robots?.writer || {}, e = state.robots?.executor || {}, d = state.demo_control || {};

  setText('connection', restarting ? 'Restarting · reconnecting…' : fresh() ? 'Gateway live' : connected ? 'Gateway stale' : 'Disconnected · retrying…');
  $('connection-dot').className = `led ${fresh() ? 'live' : connected ? '' : 'down'}`;
  const t = Math.floor(state.elapsed_s || 0);
  setText('elapsed', `T+${Math.floor(t / 60)}:${String(t % 60).padStart(2, '0')}`);
  $('map-empty').hidden = Boolean(state.map) || placing();
  setText('mode-badge', `${d.navigation_mode === 'guided' ? 'Guided replay' : 'Autonomous'}${d.speed ? ` · ${SPEEDS[d.speed] || d.speed}` : ''}`);
  if (!speedSelected && d.speed) { const r = document.querySelector(`input[name=speed][value=${d.speed}]`); if (r) r.checked = true; }
  if (!modeSelected && d.navigation_mode) { const r = document.querySelector(`input[name=mode][value=${d.navigation_mode}]`); if (r) r.checked = true; }

  renderRobot('writer', w);
  renderRobot('executor', e);
  renderRobot('firebot', state.robots?.firebot || {});
  renderNetwork();

  const live = (state.memories || []).filter(m => m.mission_id === 1 && m.freshness !== 'EXPIRED');
  const gasCount = live.filter(m => m.event_type === 2).length;
  const gasRequired = d.gas_hazards_required || 1;
  const victim = live.some(m => m.event_type === 1), gas = gasCount >= gasRequired, fire = live.some(m => m.event_type === 4);
  $('victim-ready').classList.toggle('ready', victim);
  $('gas-ready').classList.toggle('ready', gas);
  $('gas-ready').querySelector('span').textContent = `Gas ${gasCount}/${gasRequired}`;
  $('fire-ready').classList.toggle('ready', fire);

  setText('demo-title', PHASES[d.phase] || 'Connecting');
  $('demo-title').classList.toggle('error', d.phase === 'ERROR');
  setText('demo-message', d.message || 'Waiting for demo controls.');

  const mission = state.mission || {};
  const fleet = d.fleet || {};
  const missions = Object.values(state.missions || {});
  const done = missions.length && missions.every(m => m.status === 'MISSION COMPLETE');
  setText('mission-status', !missions.length ? 'Awaiting brief' : done ? 'MISSION COMPLETE'
    : missions.length === 1 ? (missions[0].status || 'Briefed') : `${missions.length} robots on mission`);
  setText('mission-robot', missions.length
    ? missions.map(m => `${ROBOT_NAMES[m.robot_id] || m.robot_id}${fleet[m.robot_id] ? ` (${fleet[m.robot_id].role})` : ''}`).join(' · ')
    : 'Chosen by capability');
  setText('mission-target', missions.length
    ? missions.map(m => `${m.target_event || '—'}: ${(m.status || '').toLowerCase()}`).join(' · ')
    : 'Most urgent fire and victim memories');
  const latest = missions[missions.length - 1] || mission;
  setText('mission-seeded', latest.seeded_memories != null
    ? `${latest.seeded_memories} memories${latest.map_included ? ' + Writer map' : ' · no map'}` : '—');

  // Button availability: the same rules the command post enforces server-side.
  const busy = submitting || d.busy || d.automatic || !fresh() || restarting;
  for (const b of document.querySelectorAll('[data-action]')) {
    const a = b.dataset.action;
    let off = busy;
    if (a === 'cancel_auto') off = !d.automatic || submitting;
    if (['start', 'full_demo'].includes(a) && HANDOVER.includes(w.state)) off = true;
    if (a === 'pause' && w.state === 'FAILED') off = true;
    if (a === 'fail_writer' && w.state === 'FAILED') off = true;
    if (a === 'stuck_writer' && ['FAILED', 'STUCK'].includes(w.state)) off = true;
    if (a === 'destroy_beacon' && !$('beacon-target').value) off = true;
    if (a === 'full_demo' && e.state !== 'READY') off = true;
    if (a === 'dispatch' && (!HANDOVER.includes(w.state) || !victim || !['READY', 'MISSION_FAILED'].includes(e.state))) off = true;
    b.disabled = off;
  }
  $('cancel-auto').hidden = !d.automatic;
  $('full-demo').querySelector('span').textContent = d.automatic ? 'Sequence running…' : 'Run full demo';
  $('restart').disabled = submitting || restarting;

  let stage = HANDOVER.includes(w.state) ? 2 : ['ONLINE', 'RETURNING'].includes(w.state) ? 0 : -1;
  if (victim && gas && fire && !HANDOVER.includes(w.state)) stage = 1;
  const rescue = RESCUE.map(id => state.robots?.[id]?.state).filter(Boolean);
  if (rescue.some(s => s !== 'READY')) stage = 3;
  if (done) stage = 4;
  for (let i = 0; i < 5; i++) {
    const el = $(`stage-${i}`);
    el.classList.toggle('active', i === stage && stage !== 4);
    el.classList.toggle('done', i < stage || stage === 4);
    el.setAttribute('aria-current', i === stage ? 'step' : 'false');
  }
  $('progress-bar').style.width = `${stage < 0 ? 0 : ((stage + 1) / 5) * 100}%`;

  const select = $('beacon-target'), chosen = select.value;
  const options = (state.memories || []).filter(m => !m.destroyed).sort((a, b) => a.beacon_id - b.beacon_id)
    .map(m => `<option value="${m.beacon_id}">${bid(m.beacon_id)} · ${escapeHTML((m.roles || []).join('/') || eventInfo(m.event_type).name)}</option>`).join('');
  const optionHTML = `<option value="">Choose a beacon…</option>${options}`;
  if (select.dataset.html !== optionHTML) { select.innerHTML = optionHTML; select.dataset.html = optionHTML; select.value = chosen; }

  const memories = (state.memories || []).slice()
    .filter(m => memFilter === 'all' || (memFilter === 'events' ? m.event_type !== 0 : m.event_type === 0))
    .sort((a, b) => (b.event_type !== 0) - (a.event_type !== 0) || b.event_type - a.event_type || a.beacon_id - b.beacon_id);
  setText('memory-count', String((state.memories || []).length));
  const memHTML = memories.map(memoryCard).join('') ||
    `<p class="empty">${(state.memories || []).length ? 'No memories match this filter.' : 'No memories yet. Start exploration to discover the environment.'}</p>`;
  if (memHTML !== lastMemHTML) { $('mem').innerHTML = memHTML; lastMemHTML = memHTML; }

  const timelineHTML = (d.events || []).slice().reverse()
    .map(ev => `<li><time>${escapeHTML(ev.time)}</time>${escapeHTML(ev.message)}</li>`).join('') ||
    '<li><time>—</time>Waiting for the first command.</li>';
  if (timelineHTML !== lastTimelineHTML) { $('timeline').innerHTML = timelineHTML; lastTimelineHTML = timelineHTML; }
}

/* ---------- Commands ---------- */
async function post(path, payload) {
  submitting = true; render();
  const notice = $('notice');
  try {
    const response = await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
    const result = await response.json();
    if (!response.ok) throw new Error(result.detail || 'Command rejected');
    notice.textContent = result.message || 'Command sent. Waiting for gateway acknowledgement.';
    return true;
  } catch (error) {
    notice.textContent = error.message;
    return false;
  } finally {
    submitting = false; render();
    clearTimeout(post.timer);
    post.timer = setTimeout(() => { notice.textContent = ''; }, 8000);
  }
}

for (const b of document.querySelectorAll('[data-action]')) {
  b.addEventListener('click', () => {
    const payload = { action: b.dataset.action };
    if (payload.action === 'full_demo') payload.outcome = outcome();
    if (payload.action === 'destroy_beacon') payload.beacon_id = Number($('beacon-target').value);
    post('/api/demo', payload);
  });
}
$('beacon-target').addEventListener('change', render);
$('link-cut').addEventListener('click', () => post('/api/harness/link', { up: false, restore_after_s: 20 }));
$('link-restore').addEventListener('click', () => post('/api/harness/link', { up: true }));
if (!document.querySelector('input[name=outcome]:checked')) document.querySelector('input[name=outcome]').checked = true;
$('restart').addEventListener('click', async () => {
  const speed = document.querySelector('input[name=speed]:checked')?.value || 'normal';
  const mode = document.querySelector('input[name=mode]:checked')?.value || 'autonomous';
  const events = scenario?.events.map(({ id, x, y }) => ({ id, x, y }));
  if (await post('/api/demo/restart', { mode, speed, ...(events ? { events } : {}) })) {
    $('placement-type').value = '';
    showPlacement();
    restarting = true; occupancy = null; mapKey = ''; view = null; render();
  }
});
for (const r of document.querySelectorAll('input[name=speed]')) r.addEventListener('change', () => { speedSelected = true; });
for (const r of document.querySelectorAll('input[name=mode]')) r.addEventListener('change', () => { modeSelected = true; });
for (const b of document.querySelectorAll('[data-layer]')) {
  b.addEventListener('click', () => {
    layers[b.dataset.layer] = !layers[b.dataset.layer];
    b.setAttribute('aria-pressed', String(layers[b.dataset.layer]));
  });
}
for (const b of document.querySelectorAll('[data-filter]')) {
  b.addEventListener('click', () => {
    memFilter = b.dataset.filter;
    for (const o of document.querySelectorAll('[data-filter]')) o.setAttribute('aria-pressed', String(o === b));
    render();
  });
}

function connect() {
  const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  ws.onopen = () => { connected = true; };
  ws.onmessage = event => { state = JSON.parse(event.data); connected = true; restarting = false; render(); };
  ws.onerror = () => ws.close();
  ws.onclose = () => { connected = false; render(); setTimeout(connect, 1000); };
}

view = contentBounds();
setInterval(render, 1000);
requestAnimationFrame(draw);
connect();
