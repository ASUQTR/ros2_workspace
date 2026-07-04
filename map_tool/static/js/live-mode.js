/* Live mode: connects the browser to the backend /ws/live bridge, appends
 * incoming odometry to the renderer, and reconnects to the backend with
 * exponential backoff. (Backend<->rosbridge reconnection is handled server
 * side; its state arrives here as {type:"status"} frames.) */
(function () {
  "use strict";

  const ROLLING_BUFFER = 50; // points retained across a transient backend drop

  class LiveMode {
    constructor(renderer, settings) {
      this.renderer = renderer;
      this.settings = settings;
      this.ws = null;
      this.active = false;
      this._backoff = 0;
      this._reconnectTimer = null;
      this._recent = [];
      this._count = 0;
      this._rateWindow = [];
      this._wire();
    }

    _wire() {
      document.getElementById("live-connect").addEventListener("click", () => this.connect());
      document.getElementById("live-disconnect").addEventListener("click", () => this.disconnect());
      document.getElementById("live-clear").addEventListener("click", () => {
        this._recent = [];
        this._count = 0;
        this.renderer.clear();
        this._updateStats(null);
      });
    }

    _url() {
      const proto = location.protocol === "https:" ? "wss:" : "ws:";
      const params = new URLSearchParams();
      if (this.settings.rosIp) params.set("ip", this.settings.rosIp);
      if (this.settings.rosPort) params.set("port", this.settings.rosPort);
      const key = Utils.getApiKey();
      if (key) params.set("key", key);
      return `${proto}//${location.host}/ws/live?${params.toString()}`;
    }

    connect() {
      this.active = true;
      this._backoff = 0;
      this._openSocket();
      this._setButtons(true);
    }

    _openSocket() {
      if (this.ws) {
        try { this.ws.close(); } catch (e) { /* ignore */ }
      }
      window.App.setStatus("connecting", "Connecting to backend…");
      let ws;
      try {
        ws = new WebSocket(this._url());
      } catch (e) {
        this._scheduleReconnect(e.message);
        return;
      }
      this.ws = ws;

      ws.onopen = () => {
        this._backoff = 0;
      };
      ws.onmessage = (ev) => this._onMessage(ev);
      ws.onclose = () => {
        if (this.active) this._scheduleReconnect("backend connection closed");
      };
      ws.onerror = () => {
        // onclose will follow; nothing to do here.
      };
    }

    _onMessage(ev) {
      let msg;
      try {
        msg = JSON.parse(ev.data);
      } catch (e) {
        return;
      }
      if (msg.type === "odom") {
        this._onOdom(msg.data);
      } else if (msg.type === "status") {
        this._onBridgeStatus(msg.state, msg.detail);
      }
    }

    _onOdom(p) {
      this.renderer.appendPoint(p);
      this._recent.push(p);
      if (this._recent.length > ROLLING_BUFFER) this._recent.shift();
      this._count++;
      this._trackRate();
      this._updateStats(p);
    }

    _onBridgeStatus(state, detail) {
      // Reflect the backend<->rosbridge link state in the UI.
      let text = state;
      if (state === "reconnecting" && detail && detail.retry_in != null) {
        text = `ROS reconnecting in ${detail.retry_in.toFixed(0)}s…`;
      } else if (state === "connected") {
        text = "Connected to ROS bridge";
      } else if (state === "connecting") {
        text = "Connecting to ROS bridge…";
      } else if (state === "disconnected") {
        text = "ROS bridge disconnected";
      }
      window.App.setStatus(state, text);
    }

    _scheduleReconnect(reason) {
      this._setStatusReconnecting(reason);
      // Exponential backoff: 1,2,4,... capped at 30s.
      this._backoff = this._backoff < 1 ? 1 : Math.min(this._backoff * 2, 30);
      const delay = this._backoff;
      let remaining = delay;
      clearTimeout(this._reconnectTimer);
      const tick = () => {
        if (!this.active) return;
        remaining -= 1;
        if (remaining <= 0) {
          this._openSocket();
        } else {
          window.App.setStatus("reconnecting", `Reconnecting to backend in ${remaining}s…`);
          this._reconnectTimer = setTimeout(tick, 1000);
        }
      };
      window.App.setStatus("reconnecting", `Reconnecting to backend in ${remaining}s…`);
      this._reconnectTimer = setTimeout(tick, 1000);
    }

    _setStatusReconnecting(reason) {
      window.App.setStatus("reconnecting", reason || "Reconnecting…");
    }

    disconnect() {
      this.active = false;
      clearTimeout(this._reconnectTimer);
      if (this.ws) {
        try { this.ws.close(); } catch (e) { /* ignore */ }
        this.ws = null;
      }
      window.App.setStatus("idle", "Disconnected");
      this._setButtons(false);
    }

    onExit() {
      // Called when switching away from live mode.
      this.disconnect();
    }

    _setButtons(connected) {
      document.getElementById("live-connect").disabled = connected;
      document.getElementById("live-disconnect").disabled = !connected;
    }

    _trackRate() {
      const now = performance.now();
      this._rateWindow.push(now);
      while (this._rateWindow.length && now - this._rateWindow[0] > 1000) {
        this._rateWindow.shift();
      }
    }

    _updateStats(p) {
      document.getElementById("live-count").textContent = String(this._count);
      if (!p) {
        document.getElementById("live-pos").textContent = "—";
        document.getElementById("live-heading").textContent = "—";
        document.getElementById("live-rate").textContent = "—";
        return;
      }
      document.getElementById("live-pos").textContent =
        `${p.x.toFixed(2)}, ${p.y.toFixed(2)} m`;
      document.getElementById("live-heading").textContent =
        `${((p.heading * 180) / Math.PI).toFixed(1)}°`;
      document.getElementById("live-rate").textContent =
        `${this._rateWindow.length} Hz`;
    }
  }

  window.LiveMode = LiveMode;
})();
