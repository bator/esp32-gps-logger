// Runs web.html's view() on a status.json from stdin and prints the boxes
// as JSON, plus the JSON keys the page does not show (tests/test_gps_logger.py).
// Then runs the whole script against a stub document: tick() -> paint() and
// placeMap() on an answer, stale() on an error (v.dom).
// Usage: node tests/web_page_test.js web.html < status.json
const fs = require("fs");
const vm = require("vm");
const html = fs.readFileSync(process.argv[2], "utf8");
const src = html.slice(html.indexOf("<script>") + 8, html.lastIndexOf("</script>"));
const ctx = {};
vm.createContext(ctx);
vm.runInContext(src, ctx);
const text = fs.readFileSync(0, "utf8");
const d = JSON.parse(text);
const v = ctx.view(d);
const labeled = new Set(ctx.OTHER);
for (const k of ctx.SHOWN) labeled.add(k);
v.unlabeled = Object.keys(d).filter((k) => !labeled.has(k));
v.tile = ctx.tile(47.1, 18.4, 17);
v.wq = [-40, -60, -63, -67, -68, -80, -81, -100, -110, null].map((r) => ctx.wifiQ(r));
v.esc = ctx.esc("<a href=\"x\" title='y'>&</a>");
v.coordSW = ctx.view({position: "-33.85678, -151.21530", hdop: 1}).coord;
v.coordNE = ctx.view({position: "47.10000, 18.40000"}).coord;
v.logNone = ctx.logLine("up 60 utc - nmea 0/0 gpsboots 0 loops 0 sats 0/None fix No_data! hdop None " +
  "points 0 sent 0 wifi off pulses 0 mem 27000 idf None/None heap 124");
v.bootNone = ctx.logLine("boot cause None prev_up - mhz 80");
v.nullSys = ctx.view({}).sys;

// The page against a stub document (no browser in CI)
function El(id) {
  this.id = id; this.innerHTML = ""; this.className = ""; this.style = {};
  this.offsetWidth = 0; this.offsetHeight = 0; this.writes = 0;
}
function page(width) {
  const els = {}, xhrs = [], timers = [];
  const doc = {
    body: new El("body"),
    getElementById: (id) => els[id] || (els[id] = new El(id)),
  };
  const map = doc.getElementById("map");
  map.offsetWidth = width; map.offsetHeight = 360;
  function XHR() { xhrs.push(this); }
  XHR.prototype.open = function (m, u) { this.method = m; this.url = u; };
  XHR.prototype.send = function () {};
  const c = {document: doc, window: {}, XMLHttpRequest: XHR,
    setTimeout: (f, ms) => timers.push(ms), Date: Date, Math: Math, JSON: JSON};
  vm.createContext(c);
  vm.runInContext(src, c);
  return {c, els, xhrs, timers, doc, map};
}
const imgs = (h) => (h.match(/<img /g) || []).length;
// the status requests only (the day history has its own: /hist)
const st = (p) => p.xhrs.filter((x) => x.url === "/status.json");
const dom = {};
let p = page(1600);
dom.first = p.xhrs.length === 1 && p.xhrs[0].url === "/status.json";
p.xhrs[0].responseText = text;
p.xhrs[0].onload();
const h0 = p.xhrs.filter((x) => x.url.indexOf("/hist?") === 0);
dom.hist = h0.map((x) => x.url.replace(/d=\d{8}/, "d=D"));
dom.mapClass = p.map.className;
dom.tilesWide = imgs(p.doc.getElementById("tiles").innerHTML);
dom.transform = p.doc.getElementById("tiles").style.transform;
dom.acc = p.doc.getElementById("acc").style.width;
dom.live = p.doc.getElementById("live").innerHTML;
dom.body = p.doc.body.className;
dom.painted = p.doc.getElementById("tm").innerHTML === v.tm && p.doc.getElementById("log").innerHTML === v.log && p.doc.getElementById("sys").innerHTML === v.sys;
dom.timer = p.timers[0];
p.map.offsetWidth = 400;
p.c.window.onresize();
dom.tilesNarrow = imgs(p.doc.getElementById("tiles").innerHTML);
const before = p.doc.getElementById("tiles").innerHTML;
p.c.placeMap(d);                        // same tile: no rebuild
dom.kept = p.doc.getElementById("tiles").innerHTML === before;
p.c.tick();
st(p)[1].onerror();
dom.staleBody = p.doc.body.className;
dom.staleLive = p.doc.getElementById("live").innerHTML;
p.c.tick();
st(p)[2].responseText = JSON.stringify({refresh_s: 30, position: null});
st(p)[2].onload();
dom.nofix = p.map.className;
dom.timer2 = p.timers[p.timers.length - 1];
p.c.tick();
st(p)[3].responseText = "{broken";
st(p)[3].onload();
dom.brokenBody = p.doc.body.className;
p = page(1100);
p.xhrs[0].ontimeout();
dom.never = p.doc.getElementById("live").innerHTML;
v.dom = dom;

// The day history: lines since local midnight drawn as one path (a gap
// over 5 min starts a new one), the zoom fitted, a full reply asked again
// from where it got to, then only what is new
function trkLine(ms, lat, lon) {
  const s = new Date(ms).toISOString().replace(/[-:]/g, "").slice(0, 15) + "Z";
  return lat.toFixed(6) + "," + lon.toFixed(6) + ",100.0,2,1.4," + s + "\n";
}
const hd = {};
p = page(1600);
const now = Date.now(), mid = new Date(now);
mid.setHours(0, 0, 0, 0);
p.c.Date = Date;
p.xhrs[0].responseText = JSON.stringify({refresh_s: 15, position: "47.18536, 18.42730", hacc: 2});
p.xhrs[0].onload();
let hx = p.xhrs.filter((x) => x.url.indexOf("/hist?") === 0);
hd.firstUrls = hx.map((x) => x.url);
let body = "#TRK2\n" + trkLine(mid.getTime() - 60000, 47.0, 18.0);   // before midnight
const t1 = Math.max(mid.getTime() + 1000, now - 3600000);
for (let k = 0; k < 10; k++) body += trkLine(t1 + k * 1000, 47.18536 + k * 0.0003, 18.4273);
body += trkLine(t1 + 20 * 60000, 47.1845, 18.4290);                   // after a gap
hx[0].status = 200;
hx[0].responseText = body.slice(0, body.length - 7);                 // a cut line
hx[0].onload();
// answer the history requests still open: re (url) -> the reply, else 404
function answer(p, re, reply) {
  for (let n = 0; n < 8; n++) {
    const open = p.xhrs.filter((x) => x.url.indexOf("/hist?") === 0 && !x.done);
    if (!open.length) break;
    const x = open[0];
    x.done = true;
    x.status = re && re.test(x.url) ? 200 : 404;
    x.responseText = x.status === 200 ? reply : "no such day";
    x.onload();
  }
}
hx[0].done = true;
answer(p, null, "");
hd.pts1 = p.c.hist.pts.length;
hd.path1 = p.doc.getElementById("trk").innerHTML;
hd.busy1 = p.c.hist.busy;
const cut = body.length - 7;
hd.cut = cut;
p.c.histLoad();                         // a minute later: only what is new
hd.urls2 = p.xhrs.filter((x) => x.url.indexOf("/hist?") === 0 && !x.done).map((x) => x.url.replace(/d=\d{8}/, "d=D"));
answer(p, new RegExp("&o=" + cut + "$"), body.slice(cut));
hd.pts = p.c.hist.pts.length;
hd.km = p.c.hist.km;
hd.path = p.doc.getElementById("trk").innerHTML;
hd.day = p.doc.getElementById("day").innerHTML;
hd.zoom = +(p.doc.getElementById("tiles").innerHTML.match(/openstreetmap\.org\/(\d+)\//) || [])[1];
hd.trkT = p.doc.getElementById("trk").style.transform === p.doc.getElementById("tiles").style.transform;
// a full reply (HIST_REPLY bytes) is asked again from its end
p = page(1600);
p.xhrs[0].responseText = JSON.stringify({refresh_s: 15, position: "47.18536, 18.42730"});
p.xhrs[0].onload();
hx = p.xhrs.filter((x) => x.url.indexOf("/hist?") === 0);
hx[0].status = 200;
hx[0].responseText = "x".repeat(16384);
hx[0].onload();
hx = p.xhrs.filter((x) => x.url.indexOf("/hist?") === 0);
hd.again = hx.length > 1 && hx[1].url === hx[0].url.replace("&o=0", "&o=16384");
v.hist = hd;
process.stdout.write(JSON.stringify(v));
