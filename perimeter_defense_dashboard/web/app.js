/* FOB-3 C2 dashboard.
 *
 * Live data: /c2/state (std_msgs/String, JSON) through rosbridge. The page
 * keeps the received states for the timeline replay. The optional exercise
 * truth layer reads /sim/truth_markers; it is simulator information for the
 * exercise control, never mixed with the defense picture.
 */
'use strict';

const RO = {
  role: {PATROL: 'PATRULARE', INTERCEPT: 'INTERCEPTARE', GUARD: 'PAZĂ', FLANK: 'FLANC',
         INVESTIGATE: 'INVESTIGARE', SEARCH: 'CĂUTARE', BLOCK: 'BLOCARE', FOLLOW: 'URMĂRIRE',
         HOLD: 'IMOBILIZARE', ESCORT: 'ESCORTĂ'},
  status: {ALERT: 'ALERTĂ', TRACKED: 'URMĂRIT', LOST: 'PIERDUT', CAPTURED: 'CAPTURAT',
           ESCORTED: 'ESCORTAT', DETAINED: 'REȚINUT', 'NO CONTACT': 'FĂRĂ CONTACT',
           NEUTRALIZED: 'CAPTURAT'},
  cls: {HOSTILE: 'OSTIL', UNKNOWN: 'NECUNOSCUT', FRIEND: 'PROPRIU'},
  threat: {GREEN: 'VERDE', ORANGE: 'PORTOCALIU', RED: 'ROȘU'},
  months: ['IAN', 'FEB', 'MAR', 'APR', 'MAI', 'IUN', 'IUL', 'AUG', 'SEP', 'OCT', 'NOI', 'DEC'],
  side: {NORTH: 'NORD', WEST: 'VEST', EAST: 'EST', SOUTH: 'SUD'},
};
// MIL-STD-2525D: friendly land unit, reconnaissance (patrol UGV); dismounted individuals.
const SIDC = {
  ugv: '10031000001213000000',
  HOSTILE: '10062700001100000000',
  UNKNOWN: '10012700001100000000',
  FRIEND: '10032700001100000000',
};
const ACTIVE = new Set(['ALERT', 'TRACKED', 'LOST']);
const CLOSED = new Set(['DETAINED', 'NO CONTACT']);
const LAYERS = [
  ['aerial', 'Imagine aeriană', true], ['grid', 'Grilă MGRS 10 m', true],
  ['base', 'Gard, breșe, clădiri', true], ['zones', 'Zone autorizate', false],
  ['routes', 'Rute de patrulare', true], ['sensors', 'Rază senzori (10 m)', true],
  ['plans', 'Planuri Nav2', true], ['predictions', 'Predicții 6 s', true],
  ['ellipses', 'Elipse de incertitudine', true], ['truth', 'Adevăr exercițiu (SIM)', false],
];

const app = {
  cfg: null, map: null, ros: null, connected: false,
  states: [], storedT: -Infinity, current: null, live: true, replayT: null,
  selected: null, filter: 'ALL', search: '',
  seen: new Set(), alerts: [], acks: new Map(), audit: [],
  layers: {}, groups: {}, markers: new Map(), truth: [],
  operator: '',
};

/* ---------- small helpers ---------- */
const $ = (id) => document.getElementById(id);
const ll = (x, y) => [y, x];
const pad = (n, w) => String(Math.max(0, Math.round(n))).padStart(w, '0');
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'}[c]));
const store = {
  get(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : JSON.parse(v); } catch (e) { return d; } },
  set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) { /* storage unavailable */ } },
};

function dtg(date) {
  const d = date || new Date();
  return `${pad(d.getUTCDate(), 2)}${pad(d.getUTCHours(), 2)}${pad(d.getUTCMinutes(), 2)}Z ` +
         `${RO.months[d.getUTCMonth()]} ${pad(d.getUTCFullYear() % 100, 2)}`;
}
function mmss(t) {
  const s = Math.max(0, Math.floor(t));
  return `${pad(Math.floor(s / 60), 2)}:${pad(s % 60, 2)}`;
}
/* Simulation time t (s) of the state -> wall clock date. */
function wallOf(t) {
  const s = app.current;
  if (!s) return new Date();
  return new Date((s.wall - (s.t - t)) * 1000);
}

/* ---------- MGRS: the local frame is aligned with the UTM grid ---------- */
const geo = {gzd: '', sq: '', e: 0, n: 0};
function initGeo(origin) {
  const m = /^(\d{1,2}[C-X])([A-Z]{2})(\d{5})(\d{5})$/.exec(origin || '');
  if (!m) return;
  geo.gzd = m[1]; geo.sq = m[2]; geo.e = +m[3]; geo.n = +m[4];
}
function mgrsOf(x, y) {
  return `${geo.gzd} ${geo.sq} ${pad(geo.e + x, 5)} ${pad(geo.n + y, 5)}`;
}
function latLonOf(x, y) {
  try {
    const p = mgrs.toPoint(`${geo.gzd}${geo.sq}${pad(geo.e + x, 5)}${pad(geo.n + y, 5)}`);
    return `${p[1].toFixed(5)}N ${p[0].toFixed(5)}E`;
  } catch (e) { return '-'; }
}

/* ---------- symbols ---------- */
const symCache = new Map();
function symbol(sidc, opts) {
  const key = sidc + JSON.stringify(opts);
  if (!symCache.has(key)) {
    const s = new ms.Symbol(sidc, Object.assign({size: 20, outlineWidth: 2, outlineColor: 'rgba(0,0,0,0.6)'}, opts));
    symCache.set(key, {svg: s.asSVG(), size: s.getSize(), anchor: s.getAnchor()});
    if (symCache.size > 400) symCache.delete(symCache.keys().next().value);
  }
  return symCache.get(key);
}
function symIcon(sidc, opts) {
  const s = symbol(sidc, opts);
  return L.divIcon({html: s.svg, className: 'sym-icon', iconSize: [s.size.width, s.size.height],
                    iconAnchor: [s.anchor.x, s.anchor.y]});
}
function contactSidc(cls) { return SIDC[cls] || SIDC.UNKNOWN; }
function headingDeg(yaw) { return ((90 - yaw * 180 / Math.PI) % 360 + 360) % 360; }

/* ---------- map ---------- */
function setupMap() {
  const c = app.cfg;
  app.map = L.map('map', {crs: L.CRS.Simple, minZoom: 1, maxZoom: 7, zoomSnap: 0.25,
                          zoomDelta: 0.5, attributionControl: false});
  const fmin = c.fence.min, fmax = c.fence.max;
  app.map.fitBounds([ll(fmin[0] - 3, fmin[1] - 3), ll(fmax[0] + 3, fmax[1] + 3)]);
  for (const [k] of LAYERS) app.groups[k] = L.layerGroup();
  app.groups.dynamic = L.layerGroup().addTo(app.map);

  const rb = [ll(c.render.min[0], c.render.min[1]), ll(c.render.max[0], c.render.max[1])];
  app.aerial = L.imageOverlay(c.aerial.day, rb, {className: 'aerial'}).addTo(app.groups.aerial);

  drawGrid(); drawBase();
  app.map.on('mousemove', (e) => {
    const x = e.latlng.lng, y = e.latlng.lat;
    $('cursor').textContent = `${mgrsOf(x, y)}   ${latLonOf(x, y)}   x ${x.toFixed(1)} y ${y.toFixed(1)}`;
  });
  app.map.on('click', () => select(null));
}

function label(x, y, text, cls) {
  return L.marker(ll(x, y), {interactive: false, keyboard: false,
    icon: L.divIcon({className: 'map-label ' + (cls || ''), html: esc(text), iconSize: null, iconAnchor: [0, 0]})});
}
function centeredLabel(x, y, text, cls) {
  const m = label(x, y, text, cls);
  m.on('add', () => { const el = m.getElement(); if (el) el.style.transform += ' translate(-50%, -50%)'; });
  return m;
}

function drawGrid() {
  const c = app.cfg, g = app.groups.grid;
  const x0 = c.render.min[0], x1 = c.render.max[0], y0 = c.render.min[1], y1 = c.render.max[1];
  const style = {color: '#ffffff', weight: 1, opacity: 0.18, interactive: false};
  const startX = Math.ceil((geo.e + x0) / 10) * 10 - geo.e;
  const startY = Math.ceil((geo.n + y0) / 10) * 10 - geo.n;
  for (let x = startX; x <= x1; x += 10) {
    L.polyline([ll(x, y0), ll(x, y1)], style).addTo(g);
    g.addLayer(label(x + 0.3, c.fence.min[1] - 2.2, pad(geo.e + x, 5), 'grid'));
  }
  for (let y = startY; y <= y1; y += 10) {
    L.polyline([ll(x0, y), ll(x1, y)], style).addTo(g);
    g.addLayer(label(c.fence.min[0] - 5.5, y + 0.9, pad(geo.n + y, 5), 'grid'));
  }
  g.addLayer(label(c.fence.min[0] - 5.5, c.fence.min[1] - 2.2, `${geo.gzd} ${geo.sq}`, 'grid'));
}

function drawBase() {
  const c = app.cfg;
  const b = app.groups.base;
  for (const [a, z] of c.fence.segments) {
    L.polyline([ll(...a), ll(...z)], {color: '#e9ede6', weight: 2, opacity: 0.9, interactive: false}).addTo(b);
  }
  for (const br of c.breaches) {
    const pts = br.side === 'north' || br.side === 'south'
      ? [ll(br.from, br.center[1]), ll(br.to, br.center[1])]
      : [ll(br.center[0], br.from), ll(br.center[0], br.to)];
    L.polyline(pts, {color: '#ff4d40', weight: 4, dashArray: '4 4', interactive: false}).addTo(b);
    const off = br.side === 'north' ? [0, 1.4] : br.side === 'west' ? [-4.6, 0.4] : [0, -1];
    b.addLayer(label(br.center[0] + off[0] - 1.6, br.center[1] + off[1],
                     `BREȘĂ ${RO.side[br.name.toUpperCase()] || br.name.toUpperCase()}`, 'small'));
  }
  for (const r of c.buildings) {
    const det = c.detention && r.name === c.detention.building;
    L.rectangle([ll(r.min[0], r.min[1]), ll(r.max[0], r.max[1])],
      {color: det ? '#6fb7ff' : '#e9ede6', weight: 1, opacity: 0.55, fill: false, interactive: false}).addTo(b);
    const name = det ? 'AREST' : r.label.toUpperCase();
    b.addLayer(centeredLabel((r.min[0] + r.max[0]) / 2, (r.min[1] + r.max[1]) / 2, name, 'small'));
  }
  const o = c.objective;
  L.circle(ll(...o.target), {radius: o.radius, color: '#ff4d40', weight: 2, fill: false, interactive: false}).addTo(b);
  b.addLayer(centeredLabel(o.target[0], o.target[1] + o.radius + 0.8, 'OBIECTIV · DEPOZIT', 'small'));
  for (const poly of Object.values(c.zones)) {
    L.polygon(poly.map((p) => ll(...p)), {color: '#6fb7ff', weight: 1, dashArray: '3 4', fillOpacity: 0.06,
                                          interactive: false}).addTo(app.groups.zones);
  }
  for (const [robot, pts] of Object.entries(c.routes)) {
    if (!c.robots.includes(robot)) continue;
    L.polyline(pts.concat([pts[0]]).map((p) => ll(...p)),
      {color: '#6fb7ff', weight: 1, opacity: 0.5, dashArray: '6 6', interactive: false}).addTo(app.groups.routes);
  }
}

function applyLayers() {
  for (const [k] of LAYERS) {
    const on = !!app.layers[k];
    const g = app.groups[k];
    if (on && !app.map.hasLayer(g)) g.addTo(app.map);
    if (!on && app.map.hasLayer(g)) app.map.removeLayer(g);
  }
  if (app.layers.truth) subscribeTruth(); else unsubscribeTruth();
  store.set('c2.layers', app.layers);
  render();
}

/* ---------- dynamic drawing ---------- */
function ellipse(x, y, cov, k) {
  const [a, b, c] = cov;
  const tr = a + c, det = a * c - b * b;
  const l1 = tr / 2 + Math.sqrt(Math.max(tr * tr / 4 - det, 0));
  const l2 = Math.max(tr / 2 - Math.sqrt(Math.max(tr * tr / 4 - det, 0)), 1e-6);
  const ang = Math.abs(b) < 1e-9 ? (a >= c ? 0 : Math.PI / 2) : Math.atan2(l1 - a, b);
  const r1 = k * Math.sqrt(l1), r2 = k * Math.sqrt(l2);
  const pts = [];
  for (let i = 0; i < 36; i++) {
    const t = (2 * Math.PI * i) / 36;
    const ex = r1 * Math.cos(t), ey = r2 * Math.sin(t);
    pts.push(ll(x + ex * Math.cos(ang) - ey * Math.sin(ang), y + ex * Math.sin(ang) + ey * Math.cos(ang)));
  }
  return pts;
}

function render() {
  const s = app.current;
  const g = app.groups.dynamic;
  g.clearLayers();
  if (!s) return;
  const L_ = app.layers;
  const friend = getCss('--friend'), hostile = getCss('--hostile'), unknown = getCss('--unknown');

  for (const r of s.robots) {
    if (L_.sensors) L.circle(ll(r.x, r.y), {radius: 10, color: friend, weight: 1, opacity: 0.35, fillOpacity: 0.04,
                                           interactive: false}).addTo(g);
    if (L_.plans && r.plan && r.plan.length > 1) {
      L.polyline(r.plan.map((p) => ll(...p)), {color: friend, weight: 2, opacity: 0.8, dashArray: '2 5',
                                               interactive: false}).addTo(g);
    }
    const hx = r.x + 2.2 * Math.cos(r.yaw), hy = r.y + 2.2 * Math.sin(r.yaw);
    L.polyline([ll(r.x, r.y), ll(hx, hy)], {color: friend, weight: 2, interactive: false}).addTo(g);
    const m = L.marker(ll(r.x, r.y), {icon: symIcon(SIDC.ugv, {uniqueDesignation: r.callsign, type: 'UGV',
                                                              size: 22}), zIndexOffset: 500});
    m.on('click', (e) => { L.DomEvent.stopPropagation(e); select('R:' + r.name); });
    m.addTo(g);
    if (app.selected === 'R:' + r.name) selectionRing(g, r.x, r.y);
  }

  const contacts = new Map(s.contacts.map((c) => [c.id, c]));
  for (const t of s.tracks) {
    if (t.cls === 'FRIEND') continue;
    const color = t.cls === 'HOSTILE' ? hostile : unknown;
    const captured = t.state === 'NEUTRALIZED';
    if (L_.ellipses && !captured) {
      L.polygon(ellipse(t.x, t.y, t.cov, 2.0), {color, weight: 1, opacity: 0.8, fillOpacity: 0.08,
                                                dashArray: t.state === 'ALERT' ? '3 3' : null,
                                                interactive: false}).addTo(g);
    }
    if (L_.predictions && t.path && t.path.length > 1 && !captured) {
      L.polyline(t.path.map((p) => ll(...p)), {color, weight: 2, dashArray: '1 6', opacity: 0.9,
                                               interactive: false}).addTo(g);
    }
    if (t.state === 'TRACKED') {
      L.polyline([ll(t.x, t.y), ll(t.x + 2 * t.vx, t.y + 2 * t.vy)], {color, weight: 2, interactive: false}).addTo(g);
    }
    const c = contacts.get(t.id);
    const status = c ? c.status : t.state;
    const m = L.marker(ll(t.x, t.y), {icon: symIcon(contactSidc(t.cls), {size: 18,
      uniqueDesignation: `T${t.id}`, additionalInformation: RO.status[status] || status}),
      zIndexOffset: 800, opacity: t.state === 'LOST' ? 0.6 : 1});
    m.on('click', (e) => { L.DomEvent.stopPropagation(e); select('C:' + t.id); });
    m.addTo(g);
    if (app.selected === 'C:' + t.id) selectionRing(g, t.x, t.y);
  }

  if (L_.truth) {
    for (const a of app.truth) {
      L.circle(ll(a.x, a.y), {radius: 0.45, color: '#ffffff', weight: 2, fill: false, interactive: false}).addTo(g);
      g.addLayer(label(a.x + 0.6, a.y - 0.4, a.text, 'truth'));
    }
  }
}
function selectionRing(g, x, y) {
  L.circle(ll(x, y), {radius: 1.6, color: getCss('--strong'), weight: 2, fill: false, dashArray: '4 3',
                      interactive: false}).addTo(g);
}
function getCss(v) { return getComputedStyle(document.body).getPropertyValue(v).trim(); }

/* ---------- entity list ---------- */
function entities() {
  const s = app.current;
  if (!s) return [];
  const out = [];
  for (const r of s.robots) {
    out.push({key: 'R:' + r.name, kind: 'OWN', name: r.callsign, sidc: SIDC.ugv,
              meta: RO.role[r.role] + (r.track ? ` · T${r.track}` : ''), right: `${r.speed.toFixed(1)} m/s`,
              cls: 'cls-FRIEND', closed: false});
  }
  for (const c of [...s.contacts].reverse()) {
    out.push({key: 'C:' + c.id, kind: c.cls, name: c.label, sidc: contactSidc(c.cls),
              meta: `<span class="st-${c.status.replace(' ', '-')}">${RO.status[c.status] || c.status}</span> · ${esc(viaRo(c.via))}`,
              right: ACTIVE.has(c.status) ? `${c.speed.toFixed(1)} m/s` : '', cls: 'cls-' + c.cls,
              closed: CLOSED.has(c.status)});
  }
  return out;
}
function viaRo(via) {
  if (via.startsWith('BREACH ')) return 'BREȘĂ ' + (RO.side[via.slice(7)] || via.slice(7));
  if (via.startsWith('SPOT ')) return 'RAPORT ' + via.slice(5);
  return via;
}
function renderList() {
  const f = app.filter, q = app.search.trim().toUpperCase();
  const rows = entities().filter((e) => {
    if (q && !e.name.toUpperCase().includes(q)) return false;
    if (f === 'ALL') return !e.closed;
    if (f === 'CLOSED') return e.closed;
    if (f === 'OWN') return e.kind === 'OWN';
    return e.kind === f && !e.closed;
  });
  $('entities').innerHTML = rows.map((e) => `
    <div class="entity ${app.selected === e.key ? 'selected' : ''}" data-key="${e.key}">
      <div class="sym">${symbol(e.sidc, {size: 14}).svg}</div>
      <div><div class="name ${e.cls}">${esc(e.name)}</div><div class="meta">${e.meta}</div></div>
      <div class="right-col mono">${e.right}</div>
    </div>`).join('') || '<div class="entity"><div></div><div class="meta">Nicio entitate</div></div>';
  const c = app.current ? app.current.counters : null;
  $('counters').textContent = c ? `ACTIVE ${c.ACTIVE}  CAPTURAȚI ${c.CAPTURED}  REȚINUȚI ${c.DETAINED}` : '';
}

/* ---------- selection, SALUTE ---------- */
function select(key) {
  app.selected = key;
  renderList(); renderSelection(); render();
}
function findSelected() {
  const s = app.current;
  if (!s || !app.selected) return null;
  const [kind, id] = app.selected.split(':');
  if (kind === 'R') return {kind, r: s.robots.find((r) => r.name === id)};
  const cid = +id;
  return {kind, c: s.contacts.find((c) => c.id === cid), t: s.tracks.find((t) => t.id === cid)};
}
function kv(rows) {
  return '<div class="kv">' + rows.map(([k, v]) => `<div class="k">${k}</div><div class="v">${v}</div>`).join('') + '</div>';
}
function bar(p) { return `<div class="bar"><div style="width:${Math.max(0, Math.min(100, p))}%"></div></div>`; }

function renderSelection() {
  const sel = findSelected();
  const box = $('selection');
  $('salute-section').hidden = true;
  if (!sel || (sel.kind === 'R' && !sel.r) || (sel.kind === 'C' && !sel.c && !sel.t)) {
    box.innerHTML = '<div class="empty">Nicio entitate selectată</div>';
    return;
  }
  if (sel.kind === 'R') {
    const r = sel.r;
    box.innerHTML = `<div class="head">${symbol(SIDC.ugv, {size: 26, uniqueDesignation: r.callsign}).svg}
      <div class="big cls-FRIEND">${esc(r.callsign)}</div></div>` + kv([
      ['ROL', RO.role[r.role] + (r.track ? ` · T${r.track}` : '')],
      ['VITEZĂ', `${r.speed.toFixed(2)} m/s`],
      ['DIRECȚIE', `${headingDeg(r.yaw).toFixed(0)}° · ${Math.round(headingDeg(r.yaw) * 6400 / 360)} mils`],
      ['POZIȚIE MGRS', mgrsOf(r.x, r.y)],
      ['LAT / LON', latLonOf(r.x, r.y)],
      ['DIST. LA ȚINTĂ', r.goal_dist !== null ? `${r.goal_dist.toFixed(1)} m` : '-'],
      ['BATERIE<span class="sim-tag">SIM</span>', `${r.battery.toFixed(0)} %${bar(r.battery)}`],
      ['LEGĂTURĂ RADIO<span class="sim-tag">SIM</span>', `${r.link.toFixed(0)} %${bar(r.link)}`],
    ]);
    return;
  }
  const c = sel.c, t = sel.t;
  const x = t ? t.x : c.x, y = t ? t.y : c.y;
  const cls = (t && t.cls) || (c && c.cls) || 'UNKNOWN';
  const status = c ? c.status : t.state;
  const speed = t ? Math.hypot(t.vx, t.vy) : (c ? c.speed : 0);
  const obj = app.cfg.objective.target;
  const dObj = Math.hypot(x - obj[0], y - obj[1]);
  const eta = ACTIVE.has(status) && speed > 0.3 ? `${(dObj / speed).toFixed(0)} s` : '-';
  const custody = c && c.custody ? `${c.custody[0]} · ${['ÎN DRUM', 'ESCORTĂ', 'REȚINUT'][c.custody[1]]}` : '-';
  box.innerHTML = `<div class="head">${symbol(contactSidc(cls), {size: 26}).svg}
    <div class="big cls-${cls}">${esc(c ? c.label : 'T' + t.id)}</div></div>` + kv([
    ['CLASĂ', `<span class="cls-${cls}">${RO.cls[cls] || cls}</span>`],
    ['STARE', `<span class="st-${status.replace(' ', '-')}">${RO.status[status] || status}</span>`],
    ['SURSĂ', c ? esc(viaRo(c.via)) : '-'],
    ['ÎNREGISTRAT', c ? dtg(wallOf(c.registered)) : '-'],
    ['VITEZĂ', `${speed.toFixed(2)} m/s`],
    ['POZIȚIE MGRS', mgrsOf(x, y)],
    ['LAT / LON', latLonOf(x, y)],
    ['DIST. OBIECTIV', `${dObj.toFixed(1)} m`],
    ['ETA OBIECTIV', eta],
    ['ROBOT ALOCAT', c && c.robot ? esc(c.robot.replace(/ (\w+)$/, (m, r) => ' ' + (RO.role[r] || r))) : '-'],
    ['CUSTODIE', esc(custody)],
  ]);
  if (cls !== 'FRIEND') {
    $('salute-section').hidden = false;
    $('salute').innerHTML = esc(salute(c, t, x, y, status, speed)).replace(/^([SALUTE]) /gm, '<b>$1</b> ');
  }
}

function salute(c, t, x, y, status, speed) {
  const robot = c && c.robot ? c.robot.split(' ')[0] : '';
  const escort = c && c.custody ? c.custody[0] : robot;
  const obj = app.cfg.objective.target;
  const toObj = Math.hypot(x - obj[0], y - obj[1]);
  let activity;
  if (status === 'DETAINED') activity = 'Reținut în clădirea de arest.';
  else if (status === 'ESCORTED') activity = `Capturat, escortat de ${escort} spre arest.`;
  else if (status === 'CAPTURED') activity = `Capturat de ${robot || 'UGV'}, imobilizat.`;
  else if (status === 'NO CONTACT') activity = 'Contact pierdut, poziție necunoscută.';
  else if (status === 'ALERT') activity = 'Trecere prin perimetru semnalată de senzor, neconfirmată vizual.';
  else if (speed > 0.3) {
    const tv = t ? Math.atan2(t.vx, t.vy) * 180 / Math.PI : 0;
    activity = `Deplasare ${speed > 1.8 ? 'în fugă' : 'pe jos'}, ${speed.toFixed(1)} m/s, azimut ` +
               `${((tv + 360) % 360).toFixed(0)}°, la ${toObj.toFixed(0)} m de depozitul de muniție.`;
  } else activity = `Staționar, la ${toObj.toFixed(0)} m de depozitul de muniție.`;
  const when = c ? dtg(wallOf(c.registered)) : dtg();
  return [
    `S Mărime: 1 persoană.`,
    `A Activitate: ${activity}`,
    `L Locație: ${mgrsOf(x, y)}.`,
    `U Unitate: neidentificată, fără transponder (${RO.cls[(t && t.cls) || (c && c.cls)] || 'NECUNOSCUT'}).`,
    `T Timp: ${when}.`,
    `E Echipament: nedeterminat. Detectat de: ${c ? viaRo(c.via) : 'lidar UGV'}.`,
  ].join('\n');
}

/* ---------- alerts and audit ---------- */
function ingestEvents(events) {
  let fresh = false;
  for (const ev of events) {
    if (app.seen.has(ev.id)) continue;
    app.seen.add(ev.id);
    app.alerts.unshift(ev);
    if (ev.severity === 'INFO') app.acks.set(ev.id, {auto: true});
    fresh = true;
  }
  if (app.alerts.length > 300) app.alerts.length = 300;
  if (fresh) renderAlerts();
}
function renderAlerts() {
  const open = app.alerts.filter((a) => !app.acks.has(a.id)).length;
  $('alert-count').textContent = open;
  $('alert-count').classList.toggle('hot', open > 0);
  $('alerts').innerHTML = app.alerts.slice(0, 120).map((a) => {
    const acked = app.acks.has(a.id);
    return `<div class="alert ${a.severity} ${acked ? 'acked' : ''}" data-id="${a.id}">
      <div class="sev"></div>
      <div><div class="txt">${esc(a.text)}</div><div class="when">${dtg(wallOf(a.t))} · T+${mmss(a.t)}</div></div>
      <div>${acked ? '' : '<button data-ack="' + a.id + '">ACK</button>'}</div></div>`;
  }).join('');
}
function ack(id) {
  const a = app.alerts.find((x) => x.id === id);
  if (!a || app.acks.has(id)) return;
  app.acks.set(id, {op: app.operator || '-', at: dtg()});
  addAudit(`ACK "${a.text}"`);
  renderAlerts();
}
function addAudit(text) {
  const entry = `${dtg()}  ${app.operator || 'OPERATOR?'}  ${text}`;
  app.audit.unshift(entry);
  if (app.audit.length > 500) app.audit.length = 500;
  store.set('c2.audit', app.audit);
  renderAudit();
  if (app.logTopic) app.logTopic.publish(new ROSLIB.Message({data: entry}));
}
function renderAudit() {
  $('audit').innerHTML = app.audit.slice(0, 200).map((e) => `<div>${esc(e)}</div>`).join('') ||
    '<div>Jurnal gol</div>';
}

/* ---------- timeline and replay ---------- */
function renderTimeline() {
  const svg = $('tl-svg');
  const w = svg.clientWidth || 800, h = svg.clientHeight || 60;
  const st = app.states;
  if (!st.length) { svg.innerHTML = ''; return; }
  const tEnd = st[st.length - 1].t, tStart = Math.max(st[0].t, tEnd - 600);
  const span = Math.max(tEnd - tStart, 30);
  const X = (t) => ((t - tStart) / span) * (w - 20) + 10;
  let out = `<line class="axis" x1="0" y1="${h - 14}" x2="${w}" y2="${h - 14}"/>`;
  let segStart = null, segLevel = null;
  for (const s of st) {
    if (s.t < tStart) continue;
    if (s.threat !== segLevel) {
      if (segLevel && segLevel !== 'GREEN') out += `<rect class="threat-${segLevel}" x="${X(segStart)}" y="2" width="${Math.max(1, X(s.t) - X(segStart))}" height="${h - 18}"/>`;
      segStart = s.t; segLevel = s.threat;
    }
  }
  if (segLevel && segLevel !== 'GREEN') out += `<rect class="threat-${segLevel}" x="${X(segStart)}" y="2" width="${Math.max(1, X(tEnd) - X(segStart))}" height="${h - 18}"/>`;
  for (let t = Math.ceil(tStart / 60) * 60; t <= tEnd; t += 60) {
    out += `<line class="axis" x1="${X(t)}" y1="${h - 18}" x2="${X(t)}" y2="${h - 10}"/><text x="${X(t) + 3}" y="${h - 2}">T+${mmss(t)}</text>`;
  }
  const yOf = {CRITICAL: 10, WARNING: 22, INFO: 34};
  for (const a of app.alerts) {
    if (a.t < tStart) continue;
    const y = yOf[a.severity] || 34;
    out += `<rect class="ev-${a.severity}" x="${X(a.t) - 3}" y="${y - 3}" width="6" height="6" transform="rotate(45 ${X(a.t)} ${y})"><title>${esc(a.text)}</title></rect>`;
  }
  out += `<line class="now" x1="${X(tEnd)}" y1="0" x2="${X(tEnd)}" y2="${h - 14}"/>`;
  if (!app.live && app.replayT !== null) out += `<line class="cursor" x1="${X(app.replayT)}" y1="0" x2="${X(app.replayT)}" y2="${h}"/>`;
  svg.setAttribute('viewBox', `0 0 ${w} ${h}`);
  svg.innerHTML = out;
  svg._scale = {tStart, span, w};
}
function replayAt(clientX) {
  const svg = $('tl-svg'), sc = svg._scale;
  if (!sc) return;
  const rect = svg.getBoundingClientRect();
  const t = sc.tStart + ((clientX - rect.left - 10) / (sc.w - 20)) * sc.span;
  const st = app.states;
  let best = st[0];
  for (const s of st) { if (s.t <= t) best = s; else break; }
  app.live = false; app.replayT = best.t; app.current = best;
  $('live-btn').classList.remove('on');
  $('replay-badge').hidden = false;
  $('replay-badge').textContent = `REPLAY · T+${mmss(best.t)}`;
  refresh();
}
function goLive() {
  app.live = true; app.replayT = null;
  app.current = app.states[app.states.length - 1] || null;
  $('live-btn').classList.add('on');
  $('replay-badge').hidden = true;
  refresh();
}

/* ---------- state updates ---------- */
function onState(msg) {
  let s;
  try { s = JSON.parse(msg.data); } catch (e) { return; }
  s.received = Date.now();
  const st = app.states;
  if (st.length && s.t < st[st.length - 1].t - 1) { st.length = 0; app.storedT = -Infinity; }   // restarted
  // Keep about two states per second for the replay; the newest replaces the last one.
  if (!st.length || s.t - app.storedT >= 0.45) { st.push(s); app.storedT = s.t; }
  else st[st.length - 1] = s;
  while (st.length > 2400) st.shift();
  ingestEvents(s.events || []);
  app.lastState = s;
  if (app.live) { app.current = s; refresh(); } else renderTimeline();
}
function refresh() {
  const s = app.current;
  render(); renderList(); renderSelection(); renderTimeline();
  if (!s) return;
  const th = $('threat');
  th.className = 'threat ' + s.threat;
  $('threat-value').textContent = RO.threat[s.threat] || s.threat;
  $('tl-time').textContent = `T+${mmss(s.t)}`;
  $('simtime').textContent = `T+ ${mmss(s.t)} SIM`;
}

/* ---------- rosbridge ---------- */
function connect() {
  const url = `ws://${location.hostname || 'localhost'}:${app.cfg.rosbridge_port}`;
  app.ros = new ROSLIB.Ros({url});
  app.ros.on('connection', () => {
    app.connected = true;
    const state = new ROSLIB.Topic({ros: app.ros, name: '/c2/state', messageType: 'std_msgs/String',
                                    throttle_rate: 200, queue_length: 1});
    state.subscribe(onState);
    app.logTopic = new ROSLIB.Topic({ros: app.ros, name: '/c2/operator_log', messageType: 'std_msgs/String'});
    if (app.layers.truth) subscribeTruth();
    updateLink();
  });
  const retry = () => {
    app.connected = false; app.truthTopic = null; updateLink();
    setTimeout(connect, 2000);
  };
  app.ros.on('close', retry);
  app.ros.on('error', () => {});
}
function updateLink() {
  const el = $('link'), txt = $('link-text');
  const s = app.lastState;
  if (!app.connected) { el.className = 'link'; txt.textContent = 'DECONECTAT'; return; }
  const age = s ? (Date.now() - s.received) / 1000 : Infinity;
  if (age > 3) { el.className = 'link'; txt.textContent = 'FĂRĂ DATE'; return; }
  const lat = s ? Math.max(0, s.received / 1000 - s.wall) : 0;
  el.className = 'link ok';
  txt.textContent = `LEGĂTURĂ OK · ${(lat * 1000).toFixed(0)} ms`;
}
function subscribeTruth() {
  if (!app.ros || !app.connected || app.truthTopic) return;
  app.truthTopic = new ROSLIB.Topic({ros: app.ros, name: '/sim/truth_markers',
                                     messageType: 'visualization_msgs/MarkerArray', throttle_rate: 250});
  app.truthTopic.subscribe((m) => {
    const actors = new Map();
    for (const mk of m.markers) {
      if (mk.ns === 'truth_body') actors.set(mk.id, Object.assign(actors.get(mk.id) || {}, {x: mk.pose.position.x, y: mk.pose.position.y}));
      if (mk.ns === 'truth_label') actors.set(mk.id, Object.assign(actors.get(mk.id) || {}, {text: mk.text}));
    }
    app.truth = [...actors.values()].filter((a) => a.x !== undefined);
  });
}
function unsubscribeTruth() {
  if (app.truthTopic) { app.truthTopic.unsubscribe(); app.truthTopic = null; }
  app.truth = [];
}
function loadTopics() {
  if (!app.ros || !app.connected) { $('system').textContent = 'Fără conexiune rosbridge.'; return; }
  app.ros.getTopics((res) => {
    const rows = res.topics.map((t, i) => [t, res.types[i]]).sort();
    $('system').innerHTML = rows.map(([t, ty]) => `<div>${esc(t)} <span style="opacity:.6">${esc(ty)}</span></div>`).join('');
  }, () => { $('system').textContent = 'rosapi indisponibil.'; });
}

/* ---------- UI wiring ---------- */
function setupUI() {
  const c = app.cfg;
  $('unit-code').textContent = c.code;
  $('unit-name').textContent = `${c.unit} · ${c.name}`;
  document.title = `${c.code} C2`;

  app.operator = store.get('c2.operator', '');
  $('operator').value = app.operator;
  $('operator').addEventListener('change', (e) => {
    const prev = app.operator;
    app.operator = e.target.value.trim().toUpperCase();
    e.target.value = app.operator;
    store.set('c2.operator', app.operator);
    addAudit(`schimbare operator ${prev || '-'} -> ${app.operator || '-'}`);
  });

  document.querySelectorAll('#filters button').forEach((b) => b.addEventListener('click', () => {
    document.querySelectorAll('#filters button').forEach((x) => x.classList.remove('on'));
    b.classList.add('on'); app.filter = b.dataset.filter; renderList();
  }));
  $('search').addEventListener('input', (e) => { app.search = e.target.value; renderList(); });
  $('entities').addEventListener('click', (e) => {
    const row = e.target.closest('.entity[data-key]');
    if (!row) return;
    select(row.dataset.key);
    const sel = findSelected();
    const p = sel && (sel.r || sel.t || sel.c);
    if (p) app.map.panTo(ll(p.x, p.y));
  });

  $('alerts').addEventListener('click', (e) => {
    const b = e.target.closest('button[data-ack]');
    if (b) { ack(+b.dataset.ack); return; }
    const row = e.target.closest('.alert');
    if (!row) return;
    const a = app.alerts.find((x) => x.id === +row.dataset.id);
    if (a && a.track) select('C:' + a.track);
    if (a && a.pos) app.map.panTo(ll(a.pos[0], a.pos[1]));
  });
  $('ack-all').addEventListener('click', () => {
    app.alerts.filter((a) => !app.acks.has(a.id)).forEach((a) => ack(a.id));
  });
  $('salute-copy').addEventListener('click', () => {
    const text = $('salute').textContent;
    if (navigator.clipboard) navigator.clipboard.writeText(text).catch(() => {});
    addAudit('raport SALUTE copiat: ' + text.split('\n')[2]);
  });

  document.querySelectorAll('.panel-title.collapsible').forEach((t) => t.addEventListener('click', () => {
    const target = $(t.dataset.target);
    target.hidden = !target.hidden;
    t.classList.toggle('collapsed', target.hidden);
    if (t.dataset.target === 'system' && !target.hidden) loadTopics();
  }));

  const night = store.get('c2.night', false);
  setNight(night);
  $('night-btn').addEventListener('click', () => setNight(!document.body.classList.contains('night')));

  app.layers = Object.assign(Object.fromEntries(LAYERS.map(([k, , d]) => [k, d])), store.get('c2.layers', {}));
  const menu = $('layers-menu');
  menu.innerHTML = LAYERS.map(([k, name]) =>
    `<label><input type="checkbox" data-layer="${k}" ${app.layers[k] ? 'checked' : ''}>${name}</label>`).join('') +
    '<div class="note">SIM = informație de control al exercițiului, nu a sistemului.</div>';
  menu.addEventListener('change', (e) => { app.layers[e.target.dataset.layer] = e.target.checked; applyLayers(); });
  $('layers-btn').addEventListener('click', () => { menu.hidden = !menu.hidden; });

  $('live-btn').addEventListener('click', goLive);
  const track = $('tl-track');
  let dragging = false;
  track.addEventListener('mousedown', (e) => { dragging = true; replayAt(e.clientX); });
  window.addEventListener('mousemove', (e) => { if (dragging) replayAt(e.clientX); });
  window.addEventListener('mouseup', () => { dragging = false; });
  window.addEventListener('resize', renderTimeline);
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') select(null);
    if (e.key === 'l' && document.activeElement.tagName !== 'INPUT') goLive();
  });

  app.audit = store.get('c2.audit', []);
  renderAudit(); renderAlerts();
  setInterval(() => {
    $('dtg').textContent = dtg();
    updateLink();
  }, 1000);
}
function setNight(on) {
  document.body.classList.toggle('night', on);
  store.set('c2.night', on);
  if (app.aerial) app.aerial.setUrl(on ? app.cfg.aerial.night : app.cfg.aerial.day);
  symCache.clear();
  if (app.current) refresh();
}

async function main() {
  try {
    app.cfg = await (await fetch('config.json', {cache: 'no-store'})).json();
  } catch (e) {
    document.body.innerHTML = '<div style="padding:20px">config.json indisponibil: porniți dashboard_server.</div>';
    return;
  }
  initGeo(app.cfg.mgrs_origin);
  setupUI();
  setupMap();
  applyLayers();
  if (window.lucide) lucide.createIcons();
  connect();
}
main();
