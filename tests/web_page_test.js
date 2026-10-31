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
const dom = {};
let p = page(1600);
dom.first = p.xhrs.length === 1 && p.xhrs[0].url === "/status.json";
p.xhrs[0].responseText = text;
p.xhrs[0].onload();
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
p.xhrs[1].onerror();
dom.staleBody = p.doc.body.className;
dom.staleLive = p.doc.getElementById("live").innerHTML;
p.c.tick();
p.xhrs[2].responseText = JSON.stringify({refresh_s: 30, position: null});
p.xhrs[2].onload();
dom.nofix = p.map.className;
dom.timer2 = p.timers[p.timers.length - 1];
p.c.tick();
p.xhrs[3].responseText = "{broken";
p.xhrs[3].onload();
dom.brokenBody = p.doc.body.className;
p = page(1100);
p.xhrs[0].ontimeout();
dom.never = p.doc.getElementById("live").innerHTML;
v.dom = dom;
process.stdout.write(JSON.stringify(v));
