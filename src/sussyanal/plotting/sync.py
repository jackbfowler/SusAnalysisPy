"""Cross-tab live sync for the Plotly figures.

The 3-D viewer (sender) broadcasts the current (steer, shock) step indices over
a ``BroadcastChannel`` (with a ``localStorage`` + ``storage``-event fallback);
the envelope page (receiver) restyles its overlay traces. Same-origin serving
over HTTP is required (``file://`` has an opaque origin where both APIs are
blocked); without it the pages simply show their static initial state.

Scripts are injected via ``fig.write_html(..., post_script=...)``.
"""
from __future__ import annotations

import json

_CHANNEL = "sussyanal-state"

_BUS_HEAD = r"""
(function () {
  "use strict";
  var CHANNEL = """

_BUS_TAIL = r""";
  var channel = null;
  try { channel = new BroadcastChannel(CHANNEL); } catch (e) {}
  function broadcast(msg) {
    msg.t = Date.now();
    try { if (channel) channel.postMessage(msg); } catch (e) {}
    try { localStorage.setItem(CHANNEL, JSON.stringify(msg)); } catch (e) {}
  }
  function onMessage(fn) {
    try { if (channel) channel.onmessage = function (ev) { fn(ev.data); }; } catch (e) {}
    try {
      window.addEventListener("storage", function (ev) {
        if (ev.key === CHANNEL && ev.newValue) fn(JSON.parse(ev.newValue));
      });
    } catch (e) {}
  }
  function getGd() { return document.querySelector(".js-plotly-plot"); }
"""

_SENDER_JS = r"""
  var cfg = __CFG__;
  var state = { steer: cfg.midSteer, shock: cfg.midShock };

  function sliderEnd(evt) {
    var slider = evt && evt.slider;
    if (!slider) return;
    var prefix = slider.currentvalue && slider.currentvalue.prefix;
    var isShock = (prefix === cfg.shockPrefix) ||
      (!prefix && slider.steps && slider.steps.length === cfg.shock.length);
    if (isShock) {
      state = { steer: cfg.midSteer, shock: cfg.shock[slider.active] };
    } else {
      state = { steer: slider.active, shock: cfg.midShock };
    }
    broadcast({ kind: "state", steer: state.steer, shock: state.shock });
  }

  var gd = getGd();
  if (gd && gd.on) {
    gd.on("plotly_sliderend", sliderEnd);
    gd.on("plotly_sliderchange", sliderEnd);
  }
  onMessage(function (msg) {
    if (msg && msg.kind === "hello") {
      broadcast({ kind: "state", steer: state.steer, shock: state.shock });
    }
  });
  setTimeout(function () {
    broadcast({ kind: "state", steer: state.steer, shock: state.shock });
  }, 600);
"""

_RECEIVER_JS = r"""
  var cfg = __CFG__;
  var gd = getGd();

  function apply(msg) {
    if (!gd) return;
    var s = msg.steer, h = msg.shock;
    var lineI = [], lineX = [], lineY = [];
    var ptI = [], ptX = [], ptY = [];
    var mmI = [], mmX = [], mmY = [];
    var i, m, y, imin, imax;
    for (i = 0; i < cfg.metrics.length; i++) {
      m = cfg.metrics[i];
      y = m.data[s];
      if (!y) continue;
      lineI.push(m.line); lineX.push(m.x); lineY.push(y);
      ptI.push(m.point); ptX.push(m.x[h]); ptY.push(y[h]);
      imin = 0; imax = 0;
      for (var j = 1; j < y.length; j++) {
        if (y[j] < y[imin]) imin = j;
        if (y[j] > y[imax]) imax = j;
      }
      mmI.push(m.min, m.max); mmX.push(m.x[imin], m.x[imax]); mmY.push(y[imin], y[imax]);
    }
    Plotly.restyle(gd, { x: lineX, y: lineY }, lineI);
    Plotly.restyle(gd, { x: ptX, y: ptY }, ptI);
    Plotly.restyle(gd, { x: mmX, y: mmY }, mmI);
    if (cfg.steerTravel && cfg.shockTravel) {
      var txt = "Steer: " + cfg.steerTravel[s].toFixed(2) +
                " in | Shock: " + cfg.shockTravel[h].toFixed(2) + " in";
      Plotly.relayout(gd, { "annotations[0].text": txt });
    }
  }

  onMessage(function (msg) {
    if (msg && msg.kind === "state") apply(msg);
  });
  broadcast({ kind: "hello" });
"""


def _bus() -> str:
    return _BUS_HEAD + json.dumps(_CHANNEL) + _BUS_TAIL


def sender_script(
    shock_indices,
    mid_steer: int,
    mid_shock: int,
    shock_prefix: str = "Shock travel: ",
    steer_prefix: str = "Steering (rack): ",
) -> str:
    """JS for the 3-D viewer: broadcast slider state across tabs.

    ``shock_indices`` are the shock step indices the viewer's shock slider
    steps over (see ``suspension3d.viewer_indices``).
    """
    cfg = {
        "shock": [int(i) for i in shock_indices],
        "midSteer": int(mid_steer),
        "midShock": int(mid_shock),
        "shockPrefix": shock_prefix,
        "steerPrefix": steer_prefix,
    }
    return _bus() + _SENDER_JS.replace("__CFG__", json.dumps(cfg)) + "\n})();\n"


def receiver_script(sync_config) -> str:
    """JS for the envelope page: restyle overlay traces on incoming state."""
    return _bus() + _RECEIVER_JS.replace("__CFG__", json.dumps(sync_config)) + "\n})();\n"
