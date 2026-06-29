// app.js — live dashboard client. Opens an SSE stream of JSON ticks from
// the pipeline and renders scrolling Canvas charts, no chart library.

const META = new Set(["tick", "t", "latency_ns", "jitter_us", "mask"]);
const WINDOW = 10;             // seconds shown
const ACCENT = "#2f81f7";

let es = null;
let running = false;
let channels = [];             // dynamic, from the first tick
const series = {};             // name -> [{t, v}]
const charts = {};             // name -> {canvas, ctx, curEl}
let hist = [];                 // [{tick, t}] for loop-rate calc
let rx = [];                   // perf.now() of recent messages, for throughput
let latLatest = 0, jitLatest = 0, tickLatest = 0;

const $ = (id) => document.getElementById(id);

function setStatus(on) {
  const s = $("status");
  s.textContent = on ? "streaming" : "stopped";
  s.className = "status " + (on ? "on" : "off");
  const b = $("toggle");
  b.textContent = on ? "Stop" : "Start";
  b.classList.toggle("running", on);
}

function reset() {
  channels = [];
  for (const k in series) delete series[k];
  hist = []; rx = [];
  latLatest = jitLatest = tickLatest = 0;
  $("charts").innerHTML = "";
  for (const k in charts) delete charts[k];
}

function start() {
  reset();
  const cfg = $("config").value;
  const filt = $("filter").value;
  es = new EventSource(`/events?config=${cfg}&filter=${filt}`);
  es.onmessage = (e) => { try { onTick(JSON.parse(e.data)); } catch (_) {} };
  es.onerror = () => { /* keep open; server may still be spinning up */ };
  running = true;
  setStatus(true);
}

function stop() {
  if (es) { es.close(); es = null; }
  running = false;
  setStatus(false);
}

function ensureChart(name) {
  if (charts[name]) return;
  const wrap = document.createElement("div");
  wrap.className = "chart";
  wrap.innerHTML =
    `<div class="head"><span class="name">${name}</span>` +
    `<span class="cur" id="cur-${name}">—</span></div>` +
    `<canvas></canvas>`;
  $("charts").appendChild(wrap);
  const canvas = wrap.querySelector("canvas");
  charts[name] = { canvas, ctx: canvas.getContext("2d"),
                   curEl: wrap.querySelector(".cur") };
}

function onTick(d) {
  rx.push(performance.now());
  tickLatest = d.tick;
  latLatest = d.latency_ns || 0;
  jitLatest = d.jitter_us || 0;
  hist.push({ tick: d.tick, t: d.t });

  if (channels.length === 0) {
    channels = Object.keys(d).filter((k) => !META.has(k));
    channels.forEach(ensureChart);
  }
  for (const ch of channels) {
    if (!(ch in d)) continue;
    (series[ch] || (series[ch] = [])).push({ t: d.t, v: d[ch] });
  }
  // Trim to the visible window (+1s slack).
  const cutoff = d.t - WINDOW - 1;
  for (const ch of channels) {
    const s = series[ch];
    let i = 0; while (i < s.length && s[i].t < cutoff) i++;
    if (i) s.splice(0, i);
  }
  while (hist.length && hist[0].t < d.t - 2) hist.shift();
}

function sizeCanvas(c) {
  const r = c.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(r.width * dpr));
  const h = Math.max(1, Math.round(r.height * dpr));
  if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
  const ctx = c.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { w: r.width, h: r.height };
}

function drawChart(name) {
  const { canvas, ctx, curEl } = charts[name];
  const { w, h } = sizeCanvas(canvas);
  ctx.clearRect(0, 0, w, h);
  const data = series[name];
  if (!data || data.length < 2) return;

  const tNow = data[data.length - 1].t;
  const t0 = tNow - WINDOW;
  let mn = Infinity, mx = -Infinity;
  for (const p of data) if (p.t >= t0) { if (p.v < mn) mn = p.v; if (p.v > mx) mx = p.v; }
  if (mn === Infinity) { mn = 0; mx = 1; }
  if (mn === mx) { mn -= 1; mx += 1; }
  const pad = (mx - mn) * 0.12; mn -= pad; mx += pad;

  const L = 6, R = 6, T = 8, B = 6;
  const pw = w - L - R, ph = h - T - B;
  const X = (t) => L + ((t - t0) / WINDOW) * pw;
  const Y = (v) => T + (1 - (v - mn) / (mx - mn)) * ph;

  ctx.strokeStyle = "#222a35"; ctx.lineWidth = 1;
  ctx.fillStyle = "#8b949e"; ctx.font = "10px ui-sans-serif, sans-serif";
  for (let i = 0; i <= 2; i++) {
    const yy = T + (i / 2) * ph;
    ctx.beginPath(); ctx.moveTo(L, yy); ctx.lineTo(w - R, yy); ctx.stroke();
  }
  ctx.fillText((mx - pad).toFixed(2), L + 2, T + 10);
  ctx.fillText((mn + pad).toFixed(2), L + 2, h - B - 2);

  ctx.strokeStyle = ACCENT; ctx.lineWidth = 1.6; ctx.beginPath();
  let started = false;
  for (const p of data) {
    if (p.t < t0) continue;
    const x = X(p.t), y = Y(p.v);
    if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
  }
  ctx.stroke();
  curEl.textContent = data[data.length - 1].v.toFixed(3);
}

function drawStats() {
  $("stat-tick").textContent = tickLatest ? tickLatest.toLocaleString() : "—";
  $("stat-lat").textContent = latLatest ? (latLatest / 1000).toFixed(2) : "—";
  $("stat-jit").textContent = jitLatest ? jitLatest.toFixed(0) : "—";

  const now = performance.now();
  while (rx.length && rx[0] < now - 1000) rx.shift();
  $("stat-rx").textContent = running ? rx.length : "—";

  if (hist.length >= 2) {
    const a = hist[0], b = hist[hist.length - 1];
    const dt = b.t - a.t, dn = b.tick - a.tick;
    $("stat-hz").textContent = dt > 0 ? (dn / dt).toFixed(0) : "—";
  } else {
    $("stat-hz").textContent = "—";
  }
}

function frame() {
  for (const ch of channels) drawChart(ch);
  drawStats();
  requestAnimationFrame(frame);
}

$("toggle").addEventListener("click", () => (running ? stop() : start()));
setStatus(false);
requestAnimationFrame(frame);
