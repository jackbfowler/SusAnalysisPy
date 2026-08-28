"""Single-page analyze output: 3-D viewer on top, live envelope below.

Assembles two Plotly figures into one HTML page and links them with in-page
JS: the 3-D viewer's shock slider moves the red current-point marker along the
envelope lines, and the steering slider switches which line is highlighted —
exactly the MATLAB behavior, with no cross-tab machinery or server required.

The viewer (``72vh``) fills the top of the page; the envelope follows below
and the page scrolls.
"""
from __future__ import annotations

import json

import plotly.io as pio

_PAGE_CSS = """
<style>
  html, body { margin: 0; padding: 0; background: #fff; }
</style>
"""

_BRIDGE_JS = r"""
(function () {
  "use strict";
  var CFG = __CFG__;
  var state = { steer: CFG.midSteer, shock: CFG.midShock };
  var gd1 = null, gd2 = null;

  function apply(msg) {
    if (!gd2) return;
    var s = msg.steer, h = msg.shock;
    var lineI = [], lineX = [], lineY = [];
    var shapeUpdate = {};
    var i, m, y, xv;
    for (i = 0; i < CFG.metrics.length; i++) {
      m = CFG.metrics[i];
      y = m.data[s];
      if (!y) continue;
      lineI.push(m.line); lineX.push(m.x); lineY.push(y);
      // current shock travel: move the subplot's vertical line (layout shape)
      xv = m.x[h];
      shapeUpdate["shapes[" + m.shape + "].x0"] = xv;
      shapeUpdate["shapes[" + m.shape + "].x1"] = xv;
    }
    Plotly.restyle(gd2, { x: lineX, y: lineY }, lineI);
    Plotly.relayout(gd2, shapeUpdate);
  }

  function sliderEnd(evt) {
    var slider = evt && evt.slider;
    if (!slider) return;
    var prefix = slider.currentvalue && slider.currentvalue.prefix;
    var isShock = (prefix === CFG.shockPrefix) ||
      (!prefix && slider.steps && slider.steps.length === CFG.shock.length);
    if (isShock) state = { steer: CFG.midSteer, shock: CFG.shock[slider.active] };
    else state = { steer: slider.active, shock: CFG.midShock };
    apply(state);
  }

  function hook() {
    gd1 = document.getElementById("viewer3d");
    gd2 = document.getElementById("envelope");
    if (!gd1 || !gd2 || !gd1.on) { setTimeout(hook, 100); return; }
    gd1.on("plotly_sliderend", sliderEnd);
    gd1.on("plotly_sliderchange", sliderEnd);
  }
  hook();
})();
"""


def analyze_page(
    viewer_fig,
    envelope_fig,
    envelope_config,
    shock_indices,
    mid_steer: int,
    mid_shock: int,
) -> str:
    """Return one HTML page: the 3-D viewer (72vh) with the live envelope below."""
    # let the to_html default sizing fill the containers; CSS-free, robust
    viewer_fig.update_layout(height=None)
    envelope_fig.update_layout(height=None)

    viewer_html = pio.to_html(
        viewer_fig,
        full_html=True,
        include_plotlyjs=True,
        div_id="viewer3d",
        default_width="100%",
        default_height="100vh",  # 3-D fills the first viewport
        auto_play=False,  # start static; sliders only move on user input
    )
    env_html = pio.to_html(
        envelope_fig,
        full_html=False,
        include_plotlyjs=False,
        div_id="envelope",
        default_width="100%",
        default_height="980px",
        auto_play=False,
    )

    cfg = dict(envelope_config)
    cfg["shock"] = [int(i) for i in shock_indices]
    cfg["midSteer"] = int(mid_steer)
    cfg["midShock"] = int(mid_shock)
    cfg["shockPrefix"] = "Shock travel: "
    cfg["steerPrefix"] = "Steering (rack): "
    bridge = _BRIDGE_JS.replace("__CFG__", json.dumps(cfg))

    page = viewer_html.replace("</head>", _PAGE_CSS + "</head>")
    page = page.replace("</body>", env_html + "\n<script>\n" + bridge + "\n</script>\n</body>")
    return page
