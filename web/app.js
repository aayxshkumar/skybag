// skybag web UI: plain JavaScript, no dependencies, everything from the local API.
"use strict";

const $ = (sel) => document.querySelector(sel);
const state = { plan: null, track: null, playing: false, t: 0, lastFrame: 0, lastView: 0, near: [] };

function qs(form, extra = {}) {
  const p = new URLSearchParams();
  for (const el of form.elements) {
    if (!el.name) continue;
    if (el.type === "checkbox") { if (el.checked) p.set(el.name, "1"); }
    else p.set(el.name, el.value);
  }
  for (const [k, v] of Object.entries(extra)) p.set(k, v);
  p.set("time", $("#clock").value);
  return p.toString();
}

async function api(path, query) {
  const r = await fetch(`/api/${path}?${query}`);
  const body = await r.json();
  if (!r.ok) throw new Error(body.error || r.statusText);
  return body;
}

const fmt = (v, d = 0) => (v === null || v === undefined || Number.isNaN(v) ? "-" : Number(v).toFixed(d));
const hhmm = (min) => `${Math.floor(min / 60)}:${String(Math.round(min % 60)).padStart(2, "0")}`;

// ------------------------------------------------------------------ tabs
for (const b of document.querySelectorAll("#tabs button")) {
  b.addEventListener("click", () => {
    document.querySelectorAll("#tabs button").forEach((x) => x.classList.toggle("on", x === b));
    document.querySelectorAll(".tab").forEach((t) => t.classList.toggle("on", t.id === b.dataset.tab));
    if (b.dataset.tab === "map") drawChart();
  });
}

// ------------------------------------------------------------------ flight plan
$("#plan-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  const q = qs(e.target);
  const out = $("#plan-summary");
  out.textContent = "planning...";
  try {
    const res = await api("plan", q);
    state.plan = res;
    const p = res.plan;
    if (!p.ok) { out.innerHTML = `<span class="bad">${p.error}</span>`; return; }
    const warn = p.warnings.map((w) => `<span class="bad">${w}</span>`).join(" ");
    out.innerHTML =
      `FL${fmt(p.cruise_ft / 100)} &middot; ${fmt(p.dist_nm)} nm &middot; ${hhmm(p.time_min)} &middot; ` +
      `trip ${fmt(p.trip_kg)} kg, contingency ${fmt(p.contingency_kg)}, reserve ${fmt(p.reserve_kg)}, ` +
      `alternate ${fmt(p.alternate_kg)}, taxi ${fmt(p.taxi_kg)}, <b>block ${fmt(p.block_kg)} kg</b> &middot; ` +
      `TOW ${fmt(p.tow_kg)} kg, LDW ${fmt(p.ldw_kg)} kg &middot; ` +
      `MSA ${fmt(res.clearance.msa_ft)} ft (${res.clearance.method}) ${warn}`;
    const rows = p.legs.map((l) =>
      `<tr><td>${l.from}</td><td>${l.to}</td><td>${l.phase}</td><td>${fmt(l.dist_nm, 1)}</td><td>${fmt(l.course)}</td>` +
      `<td>${fmt(l.heading)}</td><td>${fmt(l.wind_from).padStart(3, "0")}/${fmt(l.wind_kt)}</td><td>${fmt(l.tas_kt)}</td>` +
      `<td>${fmt(l.gs_kt)}</td><td>${fmt(l.alt_start_ft)}-${fmt(l.alt_end_ft)}</td><td>${fmt(l.time_min, 1)}</td><td>${fmt(l.fuel_kg)}</td></tr>`);
    $("#legs").innerHTML =
      "<tr><th>from</th><th>to</th><th>phase</th><th>nm</th><th>crs</th><th>hdg</th><th>wind</th><th>tas</th><th>gs</th><th>alt ft</th><th>min</th><th>fuel kg</th></tr>" +
      rows.join("");
    drawProfile(res);
    // Prefill W&B with this plan's fuel.
    $("#wb-form").block.value = Math.round(p.block_kg);
    $("#wb-form").trip.value = Math.round(p.trip_kg);
    // Track for the moving map, airports around the route.
    state.track = await api("track", q + "&interval=10&seed=1");
    state.t = 0;
    const mid = res.path[Math.floor(res.path.length / 2)];
    const near = await api("near", `lat=${mid[0]}&lon=${mid[1]}&radius_nm=${Math.min(500, p.dist_nm / 2 + 100)}&major=1&limit=600`);
    state.near = near.airports;
    drawChart();
  } catch (err) {
    out.innerHTML = `<span class="bad">${err.message}</span>`;
  }
});

function drawProfile(res) {
  const c = $("#profile"), g = c.getContext("2d");
  g.clearRect(0, 0, c.width, c.height);
  const prof = res.profile;
  if (!prof.length) return;
  const maxX = prof[prof.length - 1][0];
  const maxY = Math.max(res.plan.cruise_ft, ...prof.map((p) => p[1])) * 1.1;
  const X = (x) => 40 + (x / maxX) * (c.width - 50), Y = (y) => c.height - 20 - (y / maxY) * (c.height - 30);
  g.fillStyle = "#3b4a3a";
  g.beginPath();
  g.moveTo(X(0), Y(0));
  for (const p of prof) g.lineTo(X(p[0]), Y(p[1]));
  g.lineTo(X(maxX), Y(0));
  g.fill();
  g.strokeStyle = "#46c2ff";
  g.lineWidth = 2;
  g.beginPath();
  prof.forEach((p, i) => (i ? g.lineTo(X(p[0]), Y(p[2])) : g.moveTo(X(p[0]), Y(p[2]))));
  g.stroke();
  g.strokeStyle = "#f2b33d";
  g.setLineDash([6, 4]);
  g.beginPath();
  g.moveTo(X(0), Y(res.clearance.msa_ft));
  g.lineTo(X(maxX), Y(res.clearance.msa_ft));
  g.stroke();
  g.setLineDash([]);
  g.fillStyle = "#8794a3";
  g.font = "12px system-ui";
  g.fillText(`${fmt(maxY / 1.1)} ft`, 2, Y(maxY / 1.1) + 4);
  g.fillText(`MSA ${fmt(res.clearance.msa_ft)}`, X(maxX) - 90, Y(res.clearance.msa_ft) - 4);
  g.fillText(`${fmt(maxX)} nm`, X(maxX) - 50, c.height - 4);
}

// ------------------------------------------------------------------ moving map
// Equirectangular around the route, longitudes unwrapped from the first point.
function projection(c, pts) {
  const lon0 = pts[0][1];
  const unwrap = (lon) => { let d = lon - lon0; while (d > 180) d -= 360; while (d < -180) d += 360; return lon0 + d; };
  let minLat = 90, maxLat = -90, minLon = 1e9, maxLon = -1e9;
  for (const [la, lo] of pts) {
    const u = unwrap(lo);
    minLat = Math.min(minLat, la); maxLat = Math.max(maxLat, la);
    minLon = Math.min(minLon, u); maxLon = Math.max(maxLon, u);
  }
  const pad = 0.15 * Math.max(maxLat - minLat, maxLon - minLon, 1);
  minLat -= pad; maxLat += pad; minLon -= pad; maxLon += pad;
  const k = Math.cos(((minLat + maxLat) / 2) * Math.PI / 180);
  const sx = c.width / ((maxLon - minLon) * k), sy = c.height / (maxLat - minLat), s = Math.min(sx, sy);
  const cx = (minLon + maxLon) / 2, cy = (minLat + maxLat) / 2;
  return ([la, lo]) => [c.width / 2 + (unwrap(lo) - cx) * k * s, c.height / 2 - (la - cy) * s];
}

function drawChart() {
  const c = $("#chart"), g = c.getContext("2d");
  g.clearRect(0, 0, c.width, c.height);
  if (!state.plan) {
    g.fillStyle = "#8794a3";
    g.fillText("Plan a flight first.", 20, 30);
    return;
  }
  const P = projection(c, state.plan.path);
  g.fillStyle = "#5a6878";
  g.font = "11px system-ui";
  for (const a of state.near) {
    const [x, y] = P([a.lat, a.lon]);
    g.fillRect(x - 1.5, y - 1.5, 3, 3);
    if (a.type === "large_airport") g.fillText(a.ident, x + 4, y - 3);
  }
  g.strokeStyle = "#46c2ff";
  g.lineWidth = 2;
  g.beginPath();
  state.plan.path.forEach((p, i) => { const [x, y] = P(p); i ? g.lineTo(x, y) : g.moveTo(x, y); });
  g.stroke();
  g.fillStyle = "#d8dee6";
  for (const w of state.plan.route) {
    const [x, y] = P([w.lat, w.lon]);
    g.beginPath(); g.arc(x, y, 4, 0, 2 * Math.PI); g.fill();
    g.fillText(w.name, x + 6, y + 12);
  }
  const fix = currentFix();
  if (fix) {
    const [x, y] = P([fix[1], fix[2]]);
    const a = (fix[5] * Math.PI) / 180;
    g.save();
    g.translate(x, y);
    g.rotate(a);
    g.fillStyle = "#f2b33d";
    g.beginPath(); g.moveTo(0, -9); g.lineTo(6, 7); g.lineTo(0, 3); g.lineTo(-6, 7); g.closePath(); g.fill();
    g.restore();
    $("#ownship").textContent =
      `T+${hhmm(fix[0] / 60)}  ${fix[1].toFixed(3)} ${fix[2].toFixed(3)}  FL${fmt(fix[3] / 100).padStart(3, "0")}  GS ${fmt(fix[4])} kt  TRK ${fmt(fix[5])}`;
  }
}

function currentFix() {
  const fx = state.track && state.track.fixes;
  if (!fx || !fx.length) return null;
  const i = Math.min(fx.length - 1, Math.floor(state.t / state.track.interval_s));
  return fx[i];
}

// One render at a time: the server renders on the CPU and can take seconds.
function updateView(fix) {
  if (state.viewBusy || !fix) return;
  state.viewBusy = true;
  const q = `lat=${fix[1]}&lon=${fix[2]}&alt_ft=${Math.max(fix[3], 1500)}&hdg=${fix[5]}&pitch=-4&w=400&h=225`;
  $("#view").src = `/api/view?${q}`;
}
$("#view").addEventListener("load", () => { state.viewBusy = false; });
$("#view").addEventListener("error", () => { state.viewBusy = false; });

function frame(now) {
  if (!state.playing) return;
  const dt = state.lastFrame ? (now - state.lastFrame) / 1000 : 0;
  state.lastFrame = now;
  state.t += dt * Number($("#speed").value);
  const fx = state.track.fixes;
  if (state.t > fx[fx.length - 1][0]) { state.t = fx[fx.length - 1][0]; state.playing = false; $("#play").textContent = "Play"; }
  drawChart();
  if (!state.viewBusy) updateView(currentFix());
  requestAnimationFrame(frame);
}

$("#play").addEventListener("click", () => {
  if (!state.track) return;
  state.playing = !state.playing;
  $("#play").textContent = state.playing ? "Pause" : "Play";
  state.lastFrame = 0;
  if (state.playing) { updateView(currentFix()); requestAnimationFrame(frame); }
});

// ------------------------------------------------------------------ performance
async function loadRunways() {
  const f = $("#perf-form");
  try {
    const r = await api("runways", `apt=${encodeURIComponent(f.apt.value)}&time=${$("#clock").value}`);
    const sel = f.rwy;
    const prev = sel.value;
    sel.innerHTML = r.ends.map((e) => `<option value="${e.ident}">${e.ident}${e.closed ? " (closed)" : ""}</option>`).join("");
    if (prev) sel.value = prev;
    $("#runway-list").textContent = r.ends
      .map((e) => `${e.ident}: TORA ${fmt(e.tora_m)} m, LDA ${fmt(e.lda_m)} m, ${fmt(e.heading_true)}T${e.closed ? `, CLOSED ${e.notam}` : ""}`)
      .join("  |  ");
  } catch (err) {
    $("#runway-list").textContent = err.message;
  }
}
$("#perf-form").apt.addEventListener("change", loadRunways);
$("#clock").addEventListener("change", loadRunways);

$("#perf-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  const kind = e.submitter ? e.submitter.dataset.kind : "takeoff";
  const out = $("#perf-out");
  try {
    const r = await api(kind, qs(e.target));
    const verdict = r.ok ? `<span class="ok">OK</span>` : `<span class="bad">NOT OK ${r.limit || ""}</span>`;
    if (kind === "takeoff") {
      out.innerHTML =
        `${r.airport} ${r.runway} &middot; V1 ${fmt(r.v1_kt)} VR ${fmt(r.vr_kt)} V2 ${fmt(r.v2_kt)} kt &middot; ` +
        `required ${fmt(r.required_m)} m of ${fmt(r.tora_m)} m (all engines ${fmt(r.tod_all_engines_m)}, ` +
        `engine out ${fmt(r.tod_oei_m)}, accelerate-stop ${fmt(r.asd_m)}) &middot; ` +
        `headwind credit ${fmt(r.factored_headwind_kt, 1)} kt, crosswind ${fmt(r.crosswind_kt, 1)} kt &middot; ${verdict}`;
    } else {
      out.innerHTML =
        `${r.airport} ${r.runway} &middot; VREF ${fmt(r.vref_kt)} kt &middot; actual ${fmt(r.actual_m)} m ` +
        `(air ${fmt(r.air_m)}, ground ${fmt(r.ground_m)}), required ${fmt(r.required_m)} m of LDA ${fmt(r.lda_m)} m &middot; ${verdict}`;
    }
  } catch (err) {
    out.innerHTML = `<span class="bad">${err.message}</span>`;
  }
});

// ------------------------------------------------------------------ weight and balance
$("#wb-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  const r = await api("wb", qs(e.target));
  const c = $("#envelope"), g = c.getContext("2d");
  g.clearRect(0, 0, c.width, c.height);
  const X = (cg) => 50 + ((cg - 10) / 35) * (c.width - 70), Y = (m) => c.height - 30 - ((m - 38000) / 40000) * (c.height - 50);
  g.strokeStyle = "#46c2ff";
  g.lineWidth = 2;
  g.beginPath();
  r.envelope.forEach(([m, cg], i) => (i ? g.lineTo(X(cg), Y(m)) : g.moveTo(X(cg), Y(m))));
  g.closePath();
  g.stroke();
  g.fillStyle = "#8794a3";
  g.font = "12px system-ui";
  for (let cg = 10; cg <= 45; cg += 5) g.fillText(`${cg}%`, X(cg) - 8, c.height - 10);
  for (let m = 40000; m <= 76000; m += 6000) g.fillText(`${m / 1000}t`, 8, Y(m) + 4);
  g.strokeStyle = "#f2b33d";
  g.beginPath();
  r.points.forEach((p, i) => (i ? g.lineTo(X(p.cg_mac), Y(p.mass_kg)) : g.moveTo(X(p.cg_mac), Y(p.mass_kg))));
  g.stroke();
  for (const p of r.points) {
    g.fillStyle = p.inside ? "#4cc38a" : "#ff6b5e";
    g.beginPath(); g.arc(X(p.cg_mac), Y(p.mass_kg), 5, 0, 2 * Math.PI); g.fill();
    g.fillText(p.name, X(p.cg_mac) + 8, Y(p.mass_kg) + 4);
  }
  $("#wb-out").innerHTML =
    r.points.map((p) => `${p.name} ${fmt(p.mass_kg)} kg at ${fmt(p.cg_mac, 1)}% MAC`).join("<br>") +
    "<br>" + (r.ok ? `<span class="ok">within limits</span>` : r.problems.map((x) => `<span class="bad">${x}</span>`).join("<br>"));
});

// ------------------------------------------------------------------ NOTAMs
$("#notam-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  const r = await api("notams", qs(e.target));
  $("#notam-list").innerHTML = r.notams
    .map((n) => `<div class="notam">${n.id} ${n.location} ${n.from} - ${n.to}<span class="tag ${n.active ? "on" : "off"}">${n.active ? "active" : "not active"} at ${r.time}</span>\n${n.text}</div>`)
    .join("") || "none";
});

loadRunways();
$("#plan-form").requestSubmit();
$("#notam-form").requestSubmit();
