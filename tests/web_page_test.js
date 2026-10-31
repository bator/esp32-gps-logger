// Runs web.html's view() on a status.json from stdin and prints the boxes
// as JSON, plus the JSON keys without a row label (tests/test_gps_logger.py).
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
const labeled = new Set(["device", "refresh_s", "diag", "satellites", "satellites_age_s",
                         "prev_uptime"]);
for (const g of ctx.LABELS) for (const r of g[1]) labeled.add(r[0]);
v.unlabeled = Object.keys(d).filter((k) => !labeled.has(k));
process.stdout.write(JSON.stringify(v));
