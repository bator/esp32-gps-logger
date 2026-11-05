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
// every status.json key: the page shows it, or it is for the raw JSON only
const SHOWN = ("fix sats hdop utc position nmea rx_bytes rx_age_s good_age_s gps_boots gps_loops " +
  "receiver setup_ok points left files bad_files sent upload upload_err upload_age_s days stay uptime prev_run " +
  "boot_cause temp mhz mem heap_kb idf flash_kb wifi rssi pulses fw ota prev_end fw_build").split(" ");
const OTHER = ["device", "refresh_s", "diag", "satellites", "satellites_age_s", "std", "prev_uptime", "web_errs",
  "track_drop", "write_errs", "q_thin", "pos6"];
const labeled = new Set(OTHER);
for (const k of SHOWN) labeled.add(k);
v.unlabeled = Object.keys(d).filter((k) => !labeled.has(k));
const wp = ctx.wpx(47.1, 18.4, 17);
v.tile = {x: Math.floor(wp[0] / 256), y: Math.floor(wp[1] / 256), mpp: ctx.mpp(47.1, 17)};
v.wq = [-40, -60, -63, -67, -68, -80, -81, -100, -110, null].map((r) => ctx.wifiQ(r));
v.esc = ctx.esc("<a href=\"x\" title='y'>&</a>");
v.coordSW = ctx.view({position: "-33.85678, -151.21530", hdop: 1}).coord;
v.coordNE = ctx.view({position: "47.10000, 18.40000"}).coord;
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
  XHR.prototype.overrideMimeType = function (m) { this.mime = m; };
  XHR.prototype.getResponseHeader = function (n) { return this.headers && n in this.headers ? this.headers[n] : null; };
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
dom.painted = p.doc.getElementById("tm").innerHTML === v.tm && p.doc.getElementById("sys").innerHTML === v.sys;
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
p.xhrs[0].responseText = JSON.stringify({refresh_s: 15, position: "47.18536, 18.42730", hdop: 0.8});
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
hd.km = p.c.tripsOf(p.c.hist.pts).reduce((a, t) => a + t.km, 0);
hd.kmField = "km" in p.c.hist;          // (the HUD sums the trips)
hd.path = p.doc.getElementById("trk").innerHTML;
hd.day = p.doc.getElementById("day").innerHTML;
hd.zoom = +(p.doc.getElementById("tiles").innerHTML.match(/tiles\/geoapify\/(\d+)\//) || [])[1];
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
// The new boxes: GPS link, last upload, day picker, track parse (speed),
// profile, GPX, and picking a past day on the page
const ft = {};
const c0 = page(1600).c;
const base = {uptime: 600, rx_bytes: 50000, rx_age_s: 0, good_age_s: 0, left: 0, upload_age_s: 30};
ft.linkOk = c0.gpsLink(base, {uptime: 585, rx_bytes: 44000});
ft.linkWarn = c0.gpsLink(Object.assign({}, base, {good_age_s: 7}), null).tone;
ft.linkJunk = c0.gpsLink(Object.assign({}, base, {good_age_s: 42, rx_age_s: 1}), null);
ft.linkDead = c0.gpsLink(Object.assign({}, base, {good_age_s: null, rx_age_s: null, uptime: 95}), null);
ft.linkOld = c0.gpsLink({uptime: 5}, null);
const vOk = c0.view(base, {uptime: 585, rx_bytes: 44000});
const vDead = c0.view(Object.assign({}, base, {good_age_s: 42, rx_age_s: null, left: 50, upload_age_s: 4000}), null);
ft.glOk = vOk.gl;
ft.glDead = vDead.gl;
ft.sysOk = vOk.sys;
ft.sysDead = vDead.sys;
ft.tmDead = vDead.tm;
ft.sysNone = c0.view({uptime: 10}).sys;
ft.days = c0.dayList(["20261030", "bad", "20261029", c0.utcDay(Date.now()), "20261030"]);
ft.today = c0.utcDay(Date.now());
// a past local day: its UTC files, points outside it dropped
const d0 = new Date(2026, 9, 30).getTime();
c0.hist = c0.histNew("20261030");
ft.pastDays = c0.hist.days;
ft.pastEnd = c0.hist.end - c0.hist.start;
const iso = (ms) => new Date(ms).toISOString().replace(/[-:]/g, "").slice(0, 15) + "Z";
c0.histParse(c0.hist.days[0], "#TRK2\n47.0,18.0,120.5,2,5.0," + iso(d0 + 3600000) + "\n" +
  "47.0001,18.0,121.0,2,,", "");
c0.histParse(c0.hist.days[0], iso(d0 + 3601000) + "\n" + "47.0,18.0,,3," + iso(d0 + 3602000) + "\n" +
  "47.0,18.0,1,2,1," + iso(d0 - 1000) + "\n47.0,18.0,1,2,1," + iso(d0 + 86400000) + "\n");
ft.pts = c0.hist.pts.map((q) => [q[0] - d0, q[3]]);
// a track uploaded after a newer one: its lines come later in the day file
c0.hist = c0.histNew("20261030");
const dl = (ms, la) => la + ",18.0,100.0,2,1.0," + iso(d0 + ms) + "\n";
c0.histParse(c0.hist.days[0], dl(7200000, "47.0") + dl(7201000, "47.001"));
c0.histParse(c0.hist.days[0], dl(3600000, "47.01") + dl(3601000, "47.011"));
ft.late = [c0.hist.pts.map((q) => q[0] - d0), Math.round(c0.tripsOf(c0.hist.pts).reduce((a, t) => a + t.km, 0) * 1000)];
// a 20 km drive, the live position at its end: the whole track between
// the header and the telemetry strip (zoom 13 at least cut both ends off
// under them); without a fix, a past day or a picked trip the track decides
ft.fitLive = (() => {
  c0.hist = c0.histNew(null);
  for (let k = 0; k < 1200; k++) c0.hist.pts.push([Date.now() - (1200 - k) * 1000, 47.18 + k * 0.00002, 18.30 + k * 0.00022, 120]);
  const out = [];
  for (const [w, h] of [[1100, 360], [375, 300]]) {
    const el = {offsetWidth: w, offsetHeight: h}, b = c0.mapBand(el);
    const ends = [c0.hist.pts[0], c0.hist.pts[1199]], lat = ends[1][1], lon = ends[1][2];
    const fv = c0.fitView(lat, lon, el, true, b);
    const inBand = (q) => {
      const m = c0.wpx(q[1], q[2], fv.z), x = w / 2 + (m[0] - fv.X) * fv.sc, y = h * .42 + (m[1] - fv.Y) * fv.sc;
      return x >= 16 && x <= w - 16 && y >= b.top + 12 && y <= b.bot - 12;
    };
    const nf = c0.fitView(lat, lon, el, false, b);
    out.push([fv.z + Math.log2(fv.sc) < 13, inBand(ends[0]) && inBand(ends[1]),
      Math.abs(nf.X - c0.wpx(47.18, 18.30 + 1199 * 0.00022 / 2, nf.z)[0]) < 1]);
  }
  return out;
})();
// the "now" card on the band's half away from the dot (the dot low:
// the card at the top)
ft.cardSide = (() => {
  const q = page(1100), m = q.doc.getElementById("map");
  m.offsetHeight = 300;
  q.c.hist = q.c.histNew(null);
  q.c.hist.pts.push([Date.now() - 60000, 47.20, 18.42, 100], [Date.now(), 47.18, 18.42, 100]);
  q.c.placeMap({position: "47.18, 18.42"});
  const low = q.doc.getElementById("now").className;
  q.c.hist.pts.reverse();
  q.c.placeMap({position: "47.20, 18.42"});
  const out = [low, q.doc.getElementById("now").className];
  // a phone: always at the top, the band starting under it
  const r = page(375), rm = r.doc.getElementById("map"), card = r.doc.getElementById("now");
  rm.offsetHeight = 340; card.offsetHeight = 46;
  r.c.hist = r.c.histNew(null);
  r.c.hist.pts.push([Date.now() - 60000, 47.20, 18.42, 100], [Date.now(), 47.18, 18.42, 100]);
  r.c.placeMap({position: "47.20, 18.42"});
  rm.className = "map";
  out.push(card.className, card.style.top, r.c.mapBand(rm).top);
  return out;
})();
// the card: Photon's answer, how long here, speed, heading, the address
// asked again only after ADDR_M metres and ADDR_S seconds
ft.now = (() => {
  const q = page(1100), c = q.c, out = {};
  out.addr = c.addrOf({features: [{properties: {type: "house", street: "Fűtőház utca", housenumber: "16",
    district: "Víziváros", city: "Székesfehérvár"}}]});
  out.poi = c.addrOf({features: [{properties: {type: "house", name: "Videoton", street: "Berényi út", housenumber: "72",
    city: "Székesfehérvár"}}]});
  out.street = c.addrOf({features: [{properties: {type: "street", name: "Budai út", city: "Székesfehérvár"}}]});
  out.none = c.addrOf({features: []});
  const now = Date.now(), still = [], go = [];
  for (let k = 0; k <= 120; k++) still.push([now - (120 - k) * 60000, 47.18 + (k < 3 ? 0.01 : 0.00002 * (k % 2)), 18.42, 100]);
  for (let k = 0; k <= 60; k++) go.push([now - (60 - k) * 1000, 47.18 + k * 0.00009, 18.42, 100]);   // ~10 m/s north
  c.hist = c.histNew(null);
  const hs = c.hereFor(still, 47.18, 18.42, 60, now);
  out.still = [Math.round(hs.ms / 60000), hs.kmh];
  const hg = c.hereFor(go, 47.18 + 60 * 0.00009, 18.42, 60, now);
  out.go = [hg.ms < 60000, Math.round(hg.kmh)];
  out.away = c.hereFor(still, 47.30, 18.42, 60, now).ms;
  // a stray fix 200 m off (under 2 min) keeps the stay; 5 min off ends it
  const stray = still.map((q2) => q2.slice());
  stray[60][1] = 47.182;
  out.stray = Math.round(c.hereFor(stray, 47.18, 18.42, 60, now).ms / 60000);
  for (let k = 55; k <= 60; k++) stray[k][1] = 47.182;
  out.left = Math.round(c.hereFor(stray, 47.18, 18.42, 60, now).ms / 60000);
  out.stale = c.hereFor(still.map((q2) => [q2[0] - 600000, q2[1], q2[2], q2[3]]), 47.18, 18.42, 60, now).kmh;
  c.hist = c.histNew("20261030");
  out.past = c.hereFor(still, 47.18, 18.42, 60, now).ms;
  out.headN = Math.round(c.heading(go));
  out.headE = Math.round(c.heading([[0, 47, 18], [1, 47, 18.001]]));
  out.headNone = c.heading([[0, 47, 18], [1, 47.0001, 18]]);
  out.card = c.nowCard({hdop: 1.44, good_age_s: 2}, out.addr, {ms: 7200000, kmh: 0}, "47.18000&deg; N");
  out.bare = c.nowCard({hdop: 2}, null, {ms: null, kmh: null}, "47.18000&deg; N");
  // the address requests
  const asks = () => q.xhrs.filter((x) => x.url.indexOf(c.ADDR) === 0).length;
  c.hist = c.histNew(null);
  c.addrLoad(47.18, 18.42);
  out.url = q.xhrs.filter((x) => x.url.indexOf(c.ADDR) === 0).map((x) => x.url)[0];
  const ax = q.xhrs.filter((x) => x.url.indexOf(c.ADDR) === 0)[0];
  ax.status = 200;
  ax.responseText = JSON.stringify({features: [{properties: {street: "Fűtőház utca", housenumber: "16", city: "Székesfehérvár"}}]});
  ax.onload();
  out.got = c.addr.a && c.addr.a.st;
  c.addrLoad(47.18005, 18.42);              // 5 m: not again
  const n1 = asks();
  c.addr.t = 0;
  c.addrLoad(47.18005, 18.42);              // 5 m, time passed: not again
  const n2 = asks();
  c.addrLoad(47.1805, 18.42);               // 55 m, time passed: again
  const n3 = asks();
  c.addrLoad(47.1810, 18.42);               // moved, but asked just now / busy: not again
  out.asks = [n1, n2, n3, asks()];
  // the dot moving: the cone on, turned the way it goes
  c.hist = c.histNew(null);
  go.forEach((g) => c.hist.pts.push(g));
  c.placeMap({position: (47.18 + 60 * 0.00009).toFixed(5) + ", 18.42000", hdop: 1.2});
  out.cone = [q.doc.getElementById("cone").className, q.doc.getElementById("cone").style.transform];
  return out;
})();
// the bands measured on the page: a two-line header on a phone (70 px)
// and an alert pushing the strip down; without a layout the defaults
ft.band = (() => {
  const c = page(375), rect = (t, b) => () => ({top: t, bottom: b});
  const m = c.doc.getElementById("map"), hud = c.doc.getElementById("hud"), wr = c.doc.getElementById("wrap");
  m.offsetHeight = 300;
  const d = c.c.mapBand(m);
  m.getBoundingClientRect = rect(0, 300); hud.getBoundingClientRect = rect(0, 70); wr.getBoundingClientRect = rect(250, 900);
  return [[d.top, d.bot], [c.c.mapBand(m).top, c.c.mapBand(m).bot]];
})();
// trips and stops: a 1.2 km walk (20 min), a stop, a few fixes standing
// (folded into the stop), a 0.5 km walk (10 min)
const tp = [], dLat = 1 / 110.54;
for (let k = 0; k <= 20; k++) tp.push([d0 + 8 * 3600000 + k * 60000, 47 + k * 0.06 * dLat, 18, 100]);
for (let k = 0; k < 3; k++) tp.push([d0 + 10 * 3600000 + k * 60000, 47.02, 18, 100]);
for (let k = 0; k <= 10; k++) tp.push([d0 + 11 * 3600000 + k * 60000, 47.02 + k * 0.05 * dLat, 18, 100]);
const trips = c0.tripsOf(tp);
ft.trips = trips.map((t) => [t.i0, t.i1, (t.t0 - d0) / 60000, (t.t1 - d0) / 60000, Math.round(t.km * 100) / 100]);
ft.timeline = c0.timeline(trips, 1, true);
ft.timelineNone = [c0.timeline([], null, true), c0.timeline([], null, false)];
// the places of the trips: the names, the queue
ft.placeOf = [
  {osm_key: "railway", osm_value: "station", name: "Budapest-Kelenföld", street: "Etele tér", city: "Budapest"},
  {osm_key: "building", street: "Fűtőház utca", housenumber: "16", city: "Székesfehérvár"},
  {osm_key: "amenity", name: "Gyógyszertár", street: "Balatoni út", city: "Budapest"},
  {osm_key: "highway", name: "Fűtőház utca", city: "Székesfehérvár", district: "Víziváros"},
  {osm_key: "landuse", name: "Temető", city: "Székesfehérvár", district: "Börgönd"},
  {osm_key: "place", name: "X", city: "Budapest", district: "Budapest"},
  {osm_key: "place", name: "X"}].map((x) => c0.placeOf({features: [{properties: x}]})).concat(c0.placeOf({features: []}));
ft.placeLine = c0.timeline(trips, 1, true, [["A <b>", "B"], ["", "C"]]);
ft.placeSame = c0.timeline(trips, 1, true, [["", ""], ["", ""]]) === ft.timeline;
ft.placeQ = (() => {
  const q = page(1600), c = q.c, urls = () => q.xhrs.filter((x) => x.url.indexOf(c.ADDR) === 0);
  const answer = (x, props) => {
    x.status = 200;
    x.responseText = JSON.stringify({features: props ? [{properties: props}] : []});
    x.onload();
  };
  const pts = [[0, 47.00001, 18, 100], [60000, 47.1, 18, 100], [120000, 47.10002, 18, 100], [180000, 47.5, 18, 100]];
  const tr = [{i0: 0, i1: 1}, {i0: 2, i1: 3}];
  const out = {};
  out.first = c.tripPlaces(tr, pts, true);
  out.queued = c.placeQ.map((x) => x.k);       // the open trip's end not; 47.1 once
  c.placeNext();
  c.placeNext();                               // one at a time
  out.oneAtATime = urls().length;
  out.url0 = urls()[0].url;
  answer(urls()[0], null);                     // no station: asks for what is there
  out.url1 = urls()[1].url;
  answer(urls()[1], {osm_key: "building", street: "S", city: "C"});
  out.url2 = urls()[2].url;
  answer(urls()[2], {osm_key: "railway", name: "St"});  // a station: nothing more
  out.after = [urls().length, c.placeQ.length, c.places["47.000,18.000"], c.places["47.100,18.000"]];
  out.known = c.tripPlaces(tr, pts, true);
  c.placeNext();
  out.again = urls().length;                   // known: asks nothing
  c.tripPlaces(tr, pts, false);                // the trip over: its end asked now
  c.placeNext();
  const x3 = urls()[3];
  const t0 = c.Date ? 0 : Date.now();
  x3.onerror();                                // away: no name, waits
  out.failed = [c.placeBusy, c.placeQ.length, c.placeOff > t0];
  c.placeNext();
  out.waits = urls().length;
  c.placeOff = 0;
  c.placeNext();
  urls()[4].status = 500;
  urls()[4].onload();
  out.http500 = [c.placeQ.length, c.placeOff > 0];
  c.placeOff = 0;
  c.placeNext();
  answer(urls()[5], null);
  c.placeNext();
  answer(urls()[6], null);                     // names nothing: not asked again
  out.none = [c.places["47.500,18.000"], c.placeQ.length];
  return out;
})();
ft.kpis = c0.kpis(trips);
ft.kpisLong = c0.kpis([{t0: 0, t1: 4500000, km: 12.345}]);
ft.gpx = c0.gpxText([[Date.UTC(2026, 9, 30, 8, 0, 0), 47.1, 18.4, 120.5, 5], [Date.UTC(2026, 9, 30, 8, 0, 1), 47.2, 18.5, null, null],
  [Date.UTC(2026, 9, 30, 9, 0, 0), 47.3, 18.6, 99, 0]], "GPS <day>");
// the page: a poll fills the picker; picking a past day asks for its UTC
// days once, draws the track without a fix, and does not poll it again
p = page(1600);
p.xhrs[0].responseText = JSON.stringify({refresh_s: 15, position: null, days: ["20261029", "20261030"], uptime: 60,
  rx_bytes: 0, good_age_s: null, rx_age_s: null});
p.xhrs[0].onload();
ft.nav0 = [p.doc.getElementById("dname").innerHTML, p.doc.getElementById("dprev").disabled,
  p.doc.getElementById("dnext").disabled];
ft.alert = p.doc.getElementById("gl").innerHTML;
answer(p, null, "");
p.c.step(-1);
// (east of UTC the file 20261030 also offers 20261031: step on to 30)
for (let k = 0; k < 3 && p.c.sel !== "20261030"; k++) { answer(p, null, ""); p.c.step(-1); }
const ph = p.xhrs.filter((x) => x.url.indexOf("/hist?") === 0 && !x.done);
ft.pickUrls = ph.map((x) => x.url);
const pd = new Date(2026, 9, 30, 12, 0, 0).getTime();
let pb = "#TRK2\n";
for (let k = 0; k < 5; k++) pb += "47.1" + k + ",18.4,100,2,3," + iso(pd + k * 1000) + "\n";
answer(p, new RegExp("d=" + c0.utcDay(pd) + "&o=0$"), pb);
ft.pickPts = p.c.hist.pts.length;
ft.pickDone = p.c.hist.done && !p.c.histDue();
ft.pickMap = p.map.className;
ft.pickTrk = p.doc.getElementById("trk").innerHTML.indexOf("<path") >= 0;
ft.pickDay = p.doc.getElementById("day").innerHTML;
ft.pickKpi = p.doc.getElementById("tkpi").innerHTML;
ft.pickLine = p.doc.getElementById("tline").innerHTML;
ft.nav1 = [p.doc.getElementById("dname").innerHTML, p.doc.getElementById("dprev").disabled,
  p.doc.getElementById("dnext").disabled];
p.c.hlTrip(0);
ft.hlMap = (p.doc.getElementById("trk").innerHTML.match(/<path (class="hl" )?d=/g) || []).length;
ft.hlLine = p.doc.getElementById("tline").innerHTML.indexOf('class="tr go on"') >= 0;
p.c.hlTrip(0);
ft.hlOff = (p.doc.getElementById("trk").innerHTML.match(/<path (class="hl" )?d=/g) || []).length;
for (let k = 0; k < 3 && p.c.hist.past; k++) p.c.step(1);   // (via 20261031 east of UTC)
ft.backToday = !p.c.hist.past && p.c.hist.end === Infinity;
// REV-4: at 00:30 local on Nov 2 in Budapest (23:30 UTC on Nov 1) today
// is Nov 2 and Nov 1 a past day (the UTC date made Nov 1 "today")
ft.tz = (() => {
  const saved = process.env.TZ;
  process.env.TZ = "Europe/Budapest";
  try {
    const fake = new Date(2026, 10, 2, 0, 30, 0).getTime();
    const RD = Date;
    function D(...a) { return a.length ? new RD(...a) : new RD(fake); }
    D.now = () => fake; D.UTC = RD.UTC;
    const c = {Date: D, Math: Math, JSON: JSON};
    vm.createContext(c);
    vm.runInContext(src, c);
    return [new RD(fake).getTimezoneOffset(), c.dayList(["20261031", "20261101"])];
  } finally {
    if (saved === undefined) delete process.env.TZ; else process.env.TZ = saved;
  }
})();
// REV-10: a shown day the board has deleted keeps both arrows working
ft.pruned = (() => {
  const q = page(1600);
  q.c.boardDays = ["20261028", "20261030"];
  q.c.sel = "20261029";
  q.c.navPaint();
  const nav = [q.doc.getElementById("dprev").disabled, q.doc.getElementById("dnext").disabled];
  const back = [];
  q.c.pick = (dd) => back.push(dd);
  q.c.step(-1);
  q.c.step(1);
  q.c.sel = "20261020";                  // older than every day kept
  q.c.navPaint();
  nav.push(q.doc.getElementById("dprev").disabled);
  return [nav, back];
})();
// R2: a bug drawing a box keeps LIVE (only a failed or unparsable answer
// is offline)
ft.pageErr = (() => {
  const q = page(1600);
  q.c.navPaint = () => { throw new Error("boom"); };
  q.xhrs[0].responseText = JSON.stringify({refresh_s: 15, position: null});
  q.xhrs[0].onload();
  return [q.doc.body.className, q.doc.getElementById("live").innerHTML];
})();
// WR: the page under a fixed time zone and "now" (TZ is read per call)
function inTz(tz, nowMs, fn) {
  const saved = process.env.TZ;
  process.env.TZ = tz;
  try {
    const fake = typeof nowMs === "function" ? nowMs() : nowMs;
    const RD = Date;
    function D(...a) { return a.length ? new RD(...a) : new RD(fake); }
    D.now = () => fake; D.UTC = RD.UTC; D.parse = RD.parse;
    const c = {Date: D, Math: Math, JSON: JSON};
    vm.createContext(c);
    vm.runInContext(src, c);
    return fn(c, RD);
  } finally {
    if (saved === undefined) delete process.env.TZ; else process.env.TZ = saved;
  }
}
// WR-1: an after-midnight drive in UTC+1 sits in the UTC file of the day
// before; its local day is offered and picking it reads that file
ft.wr1 = inTz("Europe/Budapest", () => new Date(2026, 10, 3, 10, 0, 0).getTime(), (c) => {
  const ds = c.dayList(["20261031", "20261103"]);
  c.hist = c.histNew("20261101");
  const files = c.hist.days.slice();
  c.histParse("20261031", "47.1,18.4,100,2,3,20261031T233000Z\n47.11,18.4,100,2,3,20261031T233100Z\n");
  return [ds, files, c.hist.pts.length];
});
// west of UTC the UTC day's first hours are the local day before
ft.wr1w = inTz("America/New_York", () => new Date(2026, 10, 3, 10, 0, 0).getTime(),
  (c) => c.dayList(["20261031", "bad", "20261031"]));
// WR-3: a past day with one point is centred on it, not on the live fix
// (one.js: ~58 000 px off the map); today with one point keeps the fix
ft.wr3 = (() => {
  const c = page(1100).c;
  c.hist = c.histNew("20261030");
  c.hist.pts.push([new Date(2026, 9, 30, 12).getTime(), 47.5, 19.05, 100]);
  const el = {offsetWidth: 1100, offsetHeight: 360};
  const fv = c.fitView(47.18, 18.42, el, true), pt = c.wpx(47.5, 19.05, fv.z);
  c.hist = c.histNew(null);
  c.hist.pts.push([Date.now(), 47.5, 19.05, 100]);
  const tv = c.fitView(47.18, 18.42, el, true), me = c.wpx(47.18, 18.42, tv.z);
  return [Math.round((pt[0] - fv.X) * fv.sc), Math.round((pt[1] - fv.Y) * fv.sc),
    Math.round(tv.X - me[0]), Math.round(tv.Y - me[1])];
})();
// WR-4: /hist offsets count bytes. A browser-like XHR: responseText is the
// body decoded as the charset the page asked for (UTF-8 by default, an
// invalid byte U+FFFD; x-user-defined: one char a byte)
ft.wr4 = (() => {
  function reply(x, buf) {
    x.done = true;
    x.status = 200;
    Object.defineProperty(x, "responseText", {get: () => /x-user-defined/.test(x.mime || "") ?
      Array.from(buf, (b) => String.fromCharCode(b < 128 ? b : 0xF700 + b)).join("") : buf.toString("utf8")});
    x.onload();
  }
  function start() {
    const q = page(1600);
    q.xhrs[0].responseText = JSON.stringify({refresh_s: 15, position: "47.18536, 18.42730"});
    q.xhrs[0].onload();
    return q;
  }
  const hx = (q) => q.xhrs.filter((x) => x.url.indexOf("/hist?") === 0);
  const q = start(), first = hx(q)[0];
  const t0 = new Date(Date.now() - 60000).toISOString().replace(/[-:]/g, "").slice(0, 15) + "Z";
  const body = Buffer.concat([Buffer.from("#TRK2\n47.1,18.4,1é,2,3," + t0 + "\n"),
    Buffer.from([0x34, 0x37, 0xff, 0x0a]), Buffer.from("47.2,18.4,100,2,3," + t0 + "\n")]);
  reply(first, body);
  answer(q, null, "");                   // the other UTC day (if any): 404
  q.c.histLoad();                        // a minute later: from where it got to
  const again = hx(q).filter((x) => !x.done && x.url.indexOf(first.url.replace(/&o=.*/, "&o=")) === 0);
  // a full reply with a two-byte char is HIST_REPLY bytes: asked again
  const q2 = start(), f2 = hx(q2)[0];
  const full = Buffer.alloc(16384, 0x78);
  full.write("é", 100, "utf8");
  reply(f2, full);
  const n2 = hx(q2).filter((x) => !x.done);
  return [body.length, again.length ? +again[0].url.replace(/.*&o=/, "") : null,
    n2.length ? n2[0].url === f2.url.replace("&o=0", "&o=16384") : null, q.c.hist.pts.length];
})();
// HIST-NEXT: a .trb day's reply is its decoded text: X-Hist-Next is where
// to ask from next (not o + the bytes received), again while short of
// X-Hist-End, a minute later from there; a reply that does not move on is
// not asked again
ft.hnext = (() => {
  const q = page(1600);
  q.xhrs[0].responseText = JSON.stringify({refresh_s: 15, position: "47.18536, 18.42730"});
  q.xhrs[0].onload();
  const D = q.c.utcDay(Date.now()), urls = [], t0 = Date.now() - 120000;
  const ln = (k) => trkLine(t0 + k * 1000, 47.18 + k * 0.0001, 18.42);
  function nextFor(day) {               // the open request for day, the others a 404
    for (let n = 0; n < 8; n++) {
      const o = q.xhrs.filter((x) => x.url.indexOf("/hist?") === 0 && !x.done);
      if (!o.length) return null;
      if (o[0].url.indexOf("d=" + day + "&") >= 0) return o[0];
      o[0].done = true;
      o[0].status = 404;
      o[0].responseText = "no such day";
      o[0].onload();
    }
    return null;
  }
  function reply(body, nx, end) {
    const x = nextFor(D);
    if (!x) return;
    x.done = true;
    x.status = 200;
    x.responseText = body;
    x.headers = {"X-Hist-Next": String(nx), "X-Hist-End": String(end)};
    urls.push(x.url.replace(/d=\d{8}/, "d=D"));
    x.onload();
  }
  reply(ln(0) + ln(1), 500, 900);       // short of the end: again from 500
  reply(ln(2), 900, 900);               // at the end
  nextFor("none");
  const pts = q.c.hist.pts.length;
  q.c.histLoad();                       // a minute later: from 900
  reply("", 900, 900);
  nextFor("none");
  q.c.histLoad();
  reply(ln(3), 900, 1400);              // no step on (a bad reply): not again
  const again = nextFor(D) !== null;
  return [urls, pts, q.c.hist.pts.length, again];
})();
// WR-7: a browser hands innerHTML back normalised (&middot; -> U+00B7):
// a poll like the last one writes no box again
ft.wr7 = (() => {
  const q = page(1600);
  const norm = (h) => String(h).replace(/&middot;/g, "·").replace(/&deg;/g, "°")
    .replace(/&plusmn;/g, "±").replace(/&#9679;/g, "●").replace(/&asymp;/g, "≈");
  const real = q.doc.getElementById;
  const els = {};
  q.doc.getElementById = (id) => {
    if (!els[id]) {
      const e = real(id);
      let h = "";
      Object.defineProperty(e, "innerHTML", {get: () => h, set: (x) => { e.writes++; h = norm(x); }});
      els[id] = e;
    }
    return els[id];
  };
  q.xhrs[0].responseText = text;
  q.xhrs[0].onload();
  const ids = ["tm", "sys", "skyh", "sig", "svt", "coord", "tkpi", "tline"];
  const w1 = ids.map((id) => els[id] ? els[id].writes : 0);
  q.c.tick();
  const x2 = st(q)[1];
  x2.responseText = text;
  x2.onload();
  const again = ids.filter((id, k) => (els[id] ? els[id].writes : 0) !== w1[k]);
  return [again, w1.every((n) => n === 1)];
})();
// TELE-FROZEN: while the receiver is silent the status lines repeat the
// last GPS time (the board's clock is the GPS); their time goes on with
// "up", so a drop-out has its real start and end, not one minute twice
ft.frozen = (() => {
  const a = c0.evNew(), L = (up, u, ok) => "up " + up + " utc " + u + " nmea " + ok + "/0 gpsboots 0 loops 0 sats 8/6 fix " +
    "3D_fix hdop 1.2 points 0 sent 0 wifi connected pulses 0 mem 17000 idf 221/208 heap 62";
  c0.evFeed(a, [L(1000, "20261102T110000Z", 600), L(1060, "20261102T110100Z", 1200), L(1120, "20261102T110100Z", 1200),
    L(1180, "20261102T110100Z", 1200), L(1240, "20261102T110100Z", 1200), L(1300, "20261102T110500Z", 1800)].join("\n") + "\n");
  return [a.t1 - a.t0, a.eps.map((e) => [e[1] - e[0], e[2]])];
})();
// WR-8: numbers from the JSON go into the HTML coerced or escaped
ft.wr8 = (() => {
  const w = c0.view({satellites: [{name: "G1", system: "GPS", svid: 1, cno: "<b>X1", elev: 45, azim: "<u>X2",
    used: true}], rssi: "<s>X3", temp: "<t>X4"});
  return Object.keys(w).filter((k) => /<[bust]>X\d/.test(w[k]));
})();
// WR-2: a 500 (a bug building status.json) shows the board's error
ft.wr2 = (() => {
  const q = page(1600);
  q.xhrs[0].status = 500;
  q.xhrs[0].responseText = "TypeError('row bug')";
  q.xhrs[0].onload();
  return [q.doc.body.className, q.doc.getElementById("live").innerHTML];
})();
// EVENT LOG: the whole diag log on the board (/diag?f=1, then f=0),
// aggregated: KPI tiles, 5-minute satellite means, events grouped by kind.
// A canned log (UTC; Budapest is UTC+1 in November): 09:00Z-11:59Z, a
// status line a minute, 3D but 30 min waiting and a 16 min gap, two after
// a boot without utc; events between them (no time of their own: the next
// status line's)
// (the receiver link: ~10 sentences a second, none in minutes 60-69 (a
// drop-out: the counter stands), 5 broken ones at minute 30; HDOP 3.5 in
// minutes 60-119, else 1.2)
const evOk = (m) => m === null ? 0 : 600 * Math.min(m, 60) + 600 * Math.max(0, m - 69);
const evS = (m, fix, sv) => {
  const t = m === null ? "-" : "20261102T" + two2(9 + Math.floor(m / 60)) + two2(m % 60) + "00Z";
  return "up " + (m === null ? 30 : 100 + m * 60) + " utc " + t + " nmea " + evOk(m) + "/" + (m !== null && m >= 30 ? 5 : 0) +
    " gpsboots 0 loops 0 sats " + sv + " fix " + fix + " hdop " + (m !== null && m >= 60 && m < 120 ? "3.5" : "1.2") +
    " points 0 sent 0 wifi connected pulses 0 mem 17000 idf 221/208 heap 62";
};
function two2(n) { return (n < 10 ? "0" : "") + n; }
const evAfter = {
  "-1": ["boot cause 1 prev_up 26731 mhz 160 mem 12912", "gps systems h135155"],
  30: ["wifi fail krumpli status 201 after 12000 ms", "wifi fail krumpli status 201 after 12010 ms (+2 more)"],
  45: ["gps reset loop 5 restarts in 6 s", "gps reset loop 3 more windows"],
  60: ["ota a3f2271 installed (was 2e8bc6e), rebooting", "boot cause 2 prev_up 3700 mhz 160 mem 9760",
    "gps systems h135155", evS(null, "Waiting_fix", "3/0"), evS(null, "Waiting_fix", "5/0"),
    "ota a3f2271 booted, checking"],
  70: ["ota a3f2271 ok"],
  80: ["gps silent 60 s, no bytes from the receiver"], 82: ["gps back after 125 s"],
  90: ["gps silent 60 s, no bytes from the receiver"], 91: ["gps back after 75 s"],
  100: ["crash up 3600 MemoryError()", "bug upload ValueError('x')", "web error KeyError('y')",
    "upload err OSError(113)", "upload aborted after 120 s", "upload ca err OSError(5)",
    "track full 20261102T1000-20261102T1005 drop 3", "time back 20261102T100500Z 20261102T100400Z",
    "track write failing 20261102_100000.trk new 20261102_100100.trk", "reboot tls memory 3",
    "gps setup not taken: no GST (2 more)", "gps refused CFGNAV,200,200,1000", "gps setup lost: no GST",
    "wifi lost krumpli rssi -62", "wifi back krumpli after 2 min 05 s, 3 failed tries, rssi -60",
    "clock set 2026-11-02T11:38:31Z", "heap grew in connect to 66 KB", "ota error OSError(-202)",
    "env SSID2: an SSID already set above, ignored", "upload API_KEY in .env: a space, control or non-ASCII character",
    "stay left wifi aps 3 scans 6", "upload low idf 40 KB"],
  130: ["ota 69f8c15 installed (was a3f2271), rebooting", "ota 69f8c15 no fix in 900 s, rolling back, bad"],
  179: ["boot cause 2 prev_up 244 mhz 160 mem 10256", "gps systems h135155"]
};
const evLog = [];
for (const l of evAfter["-1"]) evLog.push(l);
for (let m = 0; m < 180; m++) {
  if (m >= 150 && m <= 165) continue;                  // the board off: a gap
  evLog.push(evS(m, m >= 120 && m < 150 ? "Waiting_fix" : "3D_fix", m >= 120 && m < 150 ? "4/0" : "10/6"));
  for (const l of evAfter[m] || []) evLog.push(l);
}
const evText = evLog.join("\n") + "\n";
const EV = {};
try {
  // the aggregate and its boxes, fed in pieces cut anywhere (a reply ends
  // mid-line), under Budapest time with "now" that afternoon
  Object.assign(EV, inTz("Europe/Budapest", () => new Date(2026, 10, 2, 13, 30, 0).getTime(), (c) => {
    const a = c.evNew();
    for (let i = 0; i < evText.length; i += 997) c.evFeed(a, evText.slice(i, i + 997));
    const w = c.evView(a, {busy: false, done: true, cols: 36});
    const cnt = {}, spark = {}, last = {};
    for (const k in a.g) cnt[k] = a.g[k].n;
    const gs = c.evGroups(a);
    for (const g of gs) { spark[g.k] = g.spark; last[g.k] = g.last === null ? null : c.hm(g.last); }
    return {view: w, n: cnt, order: gs.map((g) => g.k), lastMs: gs.map((g) => g.last), viewN: gs.map((g) => g.n), spark: spark, last: last,
      det: Object.fromEntries(gs.map((g) => [g.k, g.det])), pend: a.pend.length,
      lines: a.n, empty: c.evView(c.evNew(), {busy: true, done: false}),
      none: c.evView(c.evNew(), {busy: false, done: true})};
  }));
  // the loader against a stub board: DIAG_OLD in two replies, DIAG_FILE,
  // then a minute later DIAG_FILE from where it got to; a smaller
  // DIAG_FILE (it rotated) reads both again; no DIAG_OLD (404) goes on
  const q = page(1600), urls = [];
  q.xhrs[0].responseText = JSON.stringify({refresh_s: 15, position: null});
  q.xhrs[0].onload();
  const dOpen = () => q.xhrs.filter((x) => x.url.indexOf("/diag?") === 0 && !x.done);
  function dReply(status, body, size) {
    const x = dOpen()[0];
    if (!x) { urls.push(null); return; }
    x.done = true;
    urls.push(x.url);
    x.status = status;
    x.responseText = body;
    x.headers = size === undefined ? {} : {"X-Diag-Size": String(size)};
    EV.mime = x.mime;
    x.onload();
  }
  const half = evLog.slice(0, 120).join("\n") + "\n", rest = evLog.slice(120).join("\n") + "\n";
  dReply(200, half.slice(0, 5000), half.length);       // DIAG_OLD, first reply
  dReply(200, half.slice(5000), half.length);          // the rest of it
  dReply(200, rest, rest.length);                      // DIAG_FILE
  EV.load1 = [q.c.dg.busy, q.c.dg.done, q.c.dg.a.n, dOpen().length];
  EV.kpi1 = q.doc.getElementById("evk").innerHTML;
  EV.grp1 = q.doc.getElementById("evg").innerHTML;
  // a minute later (the next poll): DIAG_FILE from its offset
  q.c.dg.at = 0;
  q.c.tick();
  st(q)[1].responseText = JSON.stringify({refresh_s: 15, position: null});
  st(q)[1].onload();
  const more = "wifi lost krumpli rssi -70\n" + evS(200, "3D_fix", "9/5") + "\n";
  dReply(200, more, rest.length + more.length);
  EV.load2 = [q.c.dg.a.g.wlost ? q.c.dg.a.g.wlost.n : 0, q.c.dg.off.slice()];
  // it rotated: DIAG_FILE now one line, smaller than where the page got to
  q.c.dg.at = 0;
  q.c.tick();
  st(q)[2].responseText = JSON.stringify({refresh_s: 15, position: null});
  st(q)[2].onload();
  const rot = "up 60 utc 20261102T130000Z nmea 1/0 gpsboots 0 loops 0 sats 7/4 fix 3D_fix hdop 1 points 0 sent 0 wifi connected pulses 0 mem 1 idf 1/1 heap 62\n";
  dReply(200, "", rot.length);
  const nOld = half + rest + more;
  dReply(200, nOld.slice(0, 16384), nOld.length);
  dReply(200, nOld.slice(16384), nOld.length);
  dReply(200, rot, rot.length);
  EV.rot = [q.c.dg.a.n, q.c.dg.a.g.wlost.n, q.c.dg.done, q.c.dg.busy];
  // no DIAG_OLD: a 404 goes on to DIAG_FILE
  const q2 = page(1600);
  q2.xhrs[0].responseText = JSON.stringify({refresh_s: 15, position: null});
  q2.xhrs[0].onload();
  const d2 = () => q2.xhrs.filter((x) => x.url.indexOf("/diag?") === 0 && !x.done);
  const x404 = d2()[0];
  x404.done = true; x404.status = 404; x404.responseText = "no such diag file"; x404.onload();
  const x0 = d2()[0];
  EV.no404 = [x404.url, x0 ? x0.url : null];
  // a non-ASCII SSID: one char a byte (x-user-defined), decoded as UTF-8;
  // the offset counts bytes
  const buf = Buffer.from("wifi fail café status 201 after 12000 ms\n" + evS(0, "3D_fix", "9/5") + "\n", "utf8");
  x0.done = true; x0.status = 200; x0.headers = {"X-Diag-Size": String(buf.length)};
  x0.responseText = Array.from(buf, (b) => String.fromCharCode(b < 128 ? b : 0xF700 + b)).join("");
  x0.onload();
  EV.utf = [q2.c.dg.off[0], buf.length, q2.doc.getElementById("evg").innerHTML.indexOf("café") >= 0];
  EV.urls = urls;
  EV.lens = [half.length, rest.length, more.length];
} catch (e) {
  EV.error = String(e && e.stack || e);
}
v.ev = EV;
// the kinds of diag lines (tests/test_gps_logger.py passes one of each
// format gps_logger.py writes)
if (process.env.EV_LINES) v.kinds = JSON.parse(fs.readFileSync(process.env.EV_LINES, "utf8")).map((l) => c0.evKind(l));
v.feat = ft;
process.stdout.write(JSON.stringify(v));
