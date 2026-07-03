/* Settings panel: persists ROS endpoint + API key in localStorage and merges
 * them over the backend-provided defaults from /api/config. */
(function () {
  "use strict";

  const LS = {
    ip: "mapTool.rosIp",
    port: "mapTool.rosPort",
    apiKey: "mapTool.apiKey",
  };

  class Settings {
    constructor() {
      this.defaults = { ros_bridge_ip: "", ros_bridge_port: 9090, max_upload_mb: 500 };
      this._modal = document.getElementById("settings-modal");
    }

    async init() {
      try {
        this.defaults = await Utils.api("/api/config");
      } catch (e) {
        // A 401 here means an API key is required but missing/wrong.
        this.defaults = { ros_bridge_ip: "", ros_bridge_port: 9090, max_upload_mb: 500 };
      }
      this._wire();
    }

    get rosIp() {
      return localStorage.getItem(LS.ip) || this.defaults.ros_bridge_ip || "";
    }
    get rosPort() {
      return localStorage.getItem(LS.port) || this.defaults.ros_bridge_port || 9090;
    }
    get maxUploadMb() {
      return this.defaults.max_upload_mb || 500;
    }

    _wire() {
      document.getElementById("settings-btn").addEventListener("click", () => this.open());
      document.getElementById("cfg-close").addEventListener("click", () => this.close());
      document.getElementById("cfg-save").addEventListener("click", () => this.save());
      this._modal.addEventListener("click", (e) => {
        if (e.target === this._modal) this.close();
      });
    }

    open() {
      document.getElementById("cfg-ip").value = this.rosIp;
      document.getElementById("cfg-port").value = this.rosPort;
      document.getElementById("cfg-key").value = Utils.getApiKey();
      const note = document.getElementById("cfg-note");
      note.textContent = this.defaults.api_key_required
        ? "This backend requires an API key."
        : "";
      this._modal.classList.remove("hidden");
    }

    close() {
      this._modal.classList.add("hidden");
    }

    save() {
      const ip = document.getElementById("cfg-ip").value.trim();
      const port = document.getElementById("cfg-port").value.trim();
      const key = document.getElementById("cfg-key").value;
      if (ip) localStorage.setItem(LS.ip, ip);
      else localStorage.removeItem(LS.ip);
      if (port) localStorage.setItem(LS.port, port);
      else localStorage.removeItem(LS.port);
      if (key) localStorage.setItem(LS.apiKey, key);
      else localStorage.removeItem(LS.apiKey);
      this.close();
      window.App && window.App.onSettingsChanged();
      window.App && window.App.toast("Settings saved", "ok");
    }
  }

  window.Settings = Settings;
})();
