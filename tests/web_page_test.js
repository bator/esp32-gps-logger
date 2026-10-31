// Runs web.html's view() on a status.json from stdin and prints the boxes
// as JSON, plus the JSON keys the page does not show (tests/test_gps_logger.py).
// Usage: node tests/web_page_test.js web.html < status.json
const fs = require("fs");
const vm = require("vm");
const html = fs.readFileSync(process.argv[2], "utf8");
const src = html.slice(html.indexOf("<script>") + 8, html.lastIndexOf("</script>"));
const ctx = {};
vm.createContext(ctx);
vm.runInContext(src, ctx);
const d = JSON.parse(fs.readFileSync(0, "utf8"));
const v = ctx.view(d);
const labeled = new Set(ctx.OTHER);
for (const k of ctx.SHOWN) labeled.add(k);
v.unlabeled = Object.keys(d).filter((k) => !labeled.has(k));
v.tile = ctx.tile(47.1, 18.4, 17);
v.wq = [-50, -60, -70, -80, -90, null].map((r) => ctx.wifiQ(r));
process.stdout.write(JSON.stringify(v));
