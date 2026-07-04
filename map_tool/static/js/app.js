/* App wiring: shared renderer, mode switching, status indicator, toasts. */
(function () {
  "use strict";

  const App = {
    renderer: null,
    settings: null,
    live: null,
    playback: null,
    mode: "live",
    _toastTimer: null,

    async init() {
      this.renderer = new CanvasRenderer(document.getElementById("map"));
      this.settings = new Settings();
      await this.settings.init();

      this.live = new LiveMode(this.renderer, this.settings);
      this.playback = new PlaybackMode(this.renderer, this.settings);

      document.getElementById("mode-live").addEventListener("click", () => this.setMode("live"));
      document.getElementById("mode-playback").addEventListener("click", () => this.setMode("playback"));
      document.getElementById("fit-btn").addEventListener("click", () => this.renderer.fitToBounds());

      const lastMode = localStorage.getItem("mapTool.mode") || "live";
      this.setMode(lastMode);
    },

    setMode(mode) {
      if (mode === this.mode && this._initialized) return;
      this._initialized = true;

      // Tear down the mode we are leaving.
      if (this.mode === "live" && this.live) this.live.onExit();
      if (this.mode === "playback" && this.playback) this.playback.onExit();

      this.mode = mode;
      localStorage.setItem("mapTool.mode", mode);
      this.renderer.clear();

      const isLive = mode === "live";
      document.getElementById("mode-live").classList.toggle("active", isLive);
      document.getElementById("mode-playback").classList.toggle("active", !isLive);
      document.getElementById("live-panel").classList.toggle("hidden", !isLive);
      document.getElementById("playback-panel").classList.toggle("hidden", isLive);

      this.setStatus("idle", "Idle");
      if (!isLive) this.playback.onEnter();
    },

    onSettingsChanged() {
      // If live and connected, reconnect with the new endpoint.
      if (this.mode === "live" && this.live.active) {
        this.live.disconnect();
        this.live.connect();
      }
    },

    setStatus(state, text) {
      const dot = document.getElementById("status-dot");
      dot.className = "dot dot-" + (state || "idle");
      document.getElementById("status-text").textContent = text || state || "";
    },

    toast(msg, kind) {
      const el = document.getElementById("toast");
      el.textContent = msg;
      el.className = "toast" + (kind ? " " + kind : "");
      clearTimeout(this._toastTimer);
      this._toastTimer = setTimeout(() => el.classList.add("hidden"), 4000);
    },
  };

  window.App = App;
  window.addEventListener("DOMContentLoaded", () => App.init());
})();
