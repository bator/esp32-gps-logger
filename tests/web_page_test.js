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
// Event log: the chart from two status lines, the filter chips
const evL = ["up 60 utc 20261031T053014Z nmea 9/1 gpsboots 0 loops 0 sats 10/7 fix 3D_fix hdop 1.2 points 0 sent 0 " +
  "wifi connected pulses 0 mem 27000 idf 157/120 heap 124",
  "up 120 utc 20261031T053114Z nmea 9/1 gpsboots 0 loops 0 sats 12/9 fix 2D_fix hdop 1.8 points 0 sent 0 " +
  "wifi connected pulses 0 mem 27000 idf 157/120 heap 124", "ota abc ok"];
v.evChart = ctx.evChart(ctx.evStatus(evL));
v.evDonut = ctx.evDonut(ctx.evStatus(evL), 1);
// a boot in the tail: lines without UTC (uptime only) among UTC ones, and
// none with it; the chart's x of each point (its line's end points)
const evS = (up, utc, seen) => evL[0].replace("up 60 utc 20261031T053014Z", "up " + up + " utc " + utc)
  .replace("sats 10/7", "sats " + seen + "/7");
const evX = (svg) => {
  const d = (svg.match(/class="l1" d="([^"]+)"/) || [])[1] || "";
  return d.split(/[MC]/).filter((s) => s.trim()).map((s) => { const n = s.trim().split(/\s+/); return +n[n.length - 2]; });
};
const evT = (svg) => (svg.match(/<g><text[^]*<\/g>/) || [""])[0].replace(/<[^>]+>/g, " ").trim().split(/\s+/);
v.evBoot = [[evS(600, "20261102T070000Z", 12), evS(660, "20261102T070100Z", 13), "boot cause 1 prev_up 700 mhz 80", evS(60, "-", 3)],
  [evS(60, "-", 3), evS(120, "20261102T070200Z", 9), evS(180, "20261102T070300Z", 11)],
  [evS(60, "-", 3), evS(120, "-", 5)]].map((ls) => { const s = ctx.evChart(ctx.evStatus(ls)); return [evX(s), evT(s)]; });
v.evHm = ctx.hm(Date.UTC(2026, 10, 2, 7, 0, 0));
// Avg acc from the hAcc lines only (an HDOP is no metre)
const evP = (ha) => evL[0].replace("hdop 1.2", ha ? "hdop None" : "hdop 1.2") + (ha ? " ubx 10/0 hacc " + ha : "");
v.evDonutMix = ctx.evDonut(ctx.evStatus([evP("2.0"), evP(null), evP(null)]), 0);
ctx.EVF = "er";
v.evErr = ctx.view(d).log;
v.evErrChips = ctx.view(d).evf;
ctx.EVF = "all";

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
hd.km = p.c.tripsOf(p.c.hist.pts).reduce((a, t) => a + t.km, 0);
hd.kmField = "km" in p.c.hist;          // (the HUD sums the trips)
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
// The new boxes: GPS link, last upload, day picker, track parse (speed),
// profile, GPX, static test, and picking a past day on the page
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
// a drive wider than zoom 13 shows (~20 km), the live position at its
// end: the pin stays on the map (it was off it, the view on the track's
// centre); without a fix, a past day or a picked trip the track decides
ft.fitLive = (() => {
  c0.hist = c0.histNew(null);
  for (let k = 0; k < 1200; k++) c0.hist.pts.push([Date.now() - (1200 - k) * 1000, 47.18, 18.30 + k * 0.00022, 120]);
  const out = [];
  for (const [w, h] of [[1100, 360], [375, 300]]) {
    const lat = 47.18, lon = 18.30 + 1199 * 0.00022, el = {offsetWidth: w, offsetHeight: h};
    const fv = c0.fitView(lat, lon, el, true), me = c0.wpx(lat, lon, fv.z);
    const x = w / 2 + (me[0] - fv.X) * fv.sc, y = h * .42 + (me[1] - fv.Y) * fv.sc;
    const nf = c0.fitView(lat, lon, el, false);
    out.push([fv.z, x >= 24 && x <= w - 24 && y >= 44 && y <= h - 90,
      Math.abs(nf.X - c0.wpx(lat, 18.30 + 1199 * 0.00022 / 2, nf.z)[0]) < 1]);
  }
  return out;
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
ft.kpis = c0.kpis(trips);
ft.kpisLong = c0.kpis([{t0: 0, t1: 4500000, km: 12.345}]);
ft.gpx = c0.gpxText([[Date.UTC(2026, 9, 30, 8, 0, 0), 47.1, 18.4, 120.5, 5], [Date.UTC(2026, 9, 30, 8, 0, 1), 47.2, 18.5, null, null],
  [Date.UTC(2026, 9, 30, 9, 0, 0), 47.3, 18.6, 99, 0]], "GPS <day>");
// static test: 20 fixes, 10 at the centre, 9 at 1 m, one at 10 m east
const S = [];
const ky = 110540, kx = 111320 * Math.cos(47 * Math.PI / 180);
for (let k = 0; k < 10; k++) S.push([k * 15000, 47, 18, 2]);
for (let k = 0; k < 9; k++) S.push([(10 + k) * 15000, 47 + 1 / ky, 18, 2]);
S.push([19 * 15000, 47, 18 + 10 / kx, null]);
ft.stat = c0.statView(S, true);
ft.statEmpty = c0.statView([], false);
ft.statAdd = (() => {
  c0.stat = {on: true, s: []};
  c0.statAdd({pos6: "47.185361, 18.427305", position: "47.18536, 18.42730", hacc: 1.5}, 1000);
  c0.statAdd({position: null}, 2000);
  c0.stat.on = false;
  c0.statAdd({pos6: "1, 2"}, 3000);
  return c0.stat.s;
})();
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
ft.hlMap = (p.doc.getElementById("trk").innerHTML.match(/<path/g) || []).length;
ft.hlLine = p.doc.getElementById("tline").innerHTML.indexOf('class="tr go on"') >= 0;
p.c.hlTrip(0);
ft.hlOff = (p.doc.getElementById("trk").innerHTML.match(/<path/g) || []).length;
p.c.step(1);
ft.backToday = !p.c.hist.past && p.c.hist.end === Infinity;
// REV-1: the static test keeps only a fix the receiver just sent
ft.statFresh = (() => {
  c0.stat = {on: true, s: []};
  c0.statAdd({pos6: "47.1, 18.4", good_age_s: 1}, 1);
  c0.statAdd({pos6: "47.2, 18.4", good_age_s: 8}, 2);
  c0.statAdd({pos6: "47.3, 18.4", good_age_s: null}, 3);
  return c0.stat.s.map((q) => q[1]);
})();
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
// R2: the static test is capped at STAT_MAX and keeps another tab's
// samples (it reloads the saved state before adding); a bug drawing a box
// keeps LIVE (only a failed or unparsable answer is offline)
ft.statTabs = (() => {
  const store = {};
  const ls = {getItem: (k) => (k in store ? store[k] : null), setItem: (k, v) => { store[k] = String(v); }};
  const c = {Date: Date, Math: Math, JSON: JSON, localStorage: ls};
  vm.createContext(c);
  vm.runInContext(src, c);
  c.stat = {on: true, s: []};
  c.statSave();
  c.statAdd({pos6: "47.1, 18.4", good_age_s: 0}, 1);
  // another tab adds one meanwhile
  const other = JSON.parse(store["gps-static"]);
  other.s.push([2, 47.2, 18.4, null]);
  store["gps-static"] = JSON.stringify(other);
  c.statAdd({pos6: "47.3, 18.4", good_age_s: 0}, 3);
  const merged = c.stat.s.map((q) => q[1]);
  c.stat = {on: true, s: []};
  for (let k = 0; k < c.STAT_MAX + 7; k++) c.stat.s.push([k, 47, 18, null]);
  c.statSave();
  c.statAdd({pos6: "47.4, 18.4", good_age_s: 0}, 99999);
  store["gps-static"] = "{\"on\":true,\"s\":5}";
  c.statLoad();
  return [merged, c.stat.s.length, Array.isArray(c.stat.s)];
})();
ft.pageErr = (() => {
  const q = page(1600);
  q.c.statPaint = () => { throw new Error("boom"); };
  q.xhrs[0].responseText = JSON.stringify({refresh_s: 15, position: null});
  q.xhrs[0].onload();
  return [q.doc.body.className, q.doc.getElementById("live").innerHTML];
})();
v.feat = ft;
process.stdout.write(JSON.stringify(v));
