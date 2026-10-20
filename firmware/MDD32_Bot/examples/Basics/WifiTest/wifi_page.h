/*
 * wifi_page.h - Static HTML page served by the WifiTest sketch.
 *
 * The page fetches live data from the /status JSON endpoint every
 * 3 seconds and updates the DOM, so no full-page reload is needed
 * and the sketch never templates HTML at runtime.
 */

#pragma once

const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MDD32 Bot</title>
  <style>
    body {
      font-family: system-ui, sans-serif;
      background: #1a1a2e;
      color: #e0e0e0;
      display: flex;
      justify-content: center;
      padding: 2em;
      margin: 0;
    }
    .card {
      background: #16213e;
      border-radius: 16px;
      padding: 2em;
      max-width: 360px;
      width: 100%;
    }
    h1 { margin: 0 0 .2em; color: #4fc3f7; font-size: 1.6em; }
    .sub { color: #888; margin-bottom: 1.5em; font-size: .9em; }
    .row {
      display: flex;
      justify-content: space-between;
      padding: .6em 0;
      border-bottom: 1px solid #1a1a2e;
    }
    .label { color: #aaa; }
    .value { font-weight: 600; }
    .green  { color: #66bb6a; }
    .yellow { color: #fdd835; }
  </style>
</head>
<body>
  <div class="card">
    <h1>MDD32 Bot</h1>
    <div class="sub">WiFi Test &mdash; live status</div>
    <div class="row">
      <span class="label">Uptime</span>
      <span class="value" id="uptime">--:--:--</span>
    </div>
    <div class="row">
      <span class="label">Battery</span>
      <span class="value yellow" id="voltage">-- V</span>
    </div>
    <div class="row">
      <span class="label">USB</span>
      <span class="value" id="usb">--</span>
    </div>
    <div class="row">
      <span class="label">Charging</span>
      <span class="value" id="charging">--</span>
    </div>
    <div class="row">
      <span class="label">WiFi RSSI</span>
      <span class="value" id="rssi">-- dBm</span>
    </div>
  </div>
  <script>
    function pad(n) { return String(n).padStart(2, '0'); }

    async function refresh() {
      try {
        const r = await fetch('/status');
        const d = await r.json();

        const s = d.uptime_s;
        const h = Math.floor(s / 3600);
        const m = Math.floor((s % 3600) / 60);
        document.getElementById('uptime').textContent =
          pad(h) + ':' + pad(m) + ':' + pad(s % 60);

        document.getElementById('voltage').textContent =
          d.voltage.toFixed(2) + ' V';

        const usb = document.getElementById('usb');
        usb.textContent = d.usb ? 'Connected' : 'No';
        usb.className = 'value' + (d.usb ? ' green' : '');

        const chg = document.getElementById('charging');
        chg.textContent = d.charging ? 'Yes' : 'No';
        chg.className = 'value' + (d.charging ? ' green' : '');

        document.getElementById('rssi').textContent = d.rssi + ' dBm';
      } catch (e) {
        console.error('fetch failed', e);
      }
    }

    refresh();
    setInterval(refresh, 3000);
  </script>
</body>
</html>
)rawliteral";
