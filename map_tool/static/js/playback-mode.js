/* Playback mode: upload/select a rosbag, poll extraction status, then drive a
 * timeline over the extracted odometry. Position is linearly interpolated;
 * heading uses shortest-path angular interpolation to avoid wrap artifacts. */
(function () {
  "use strict";

  const POLL_INTERVAL_MS = 500;

  class PlaybackMode {
    constructor(renderer, settings) {
      this.renderer = renderer;
      this.settings = settings;
      this.data = null; // {points, startTime, endTime, duration}
      this.currentBagId = null;
      this.t = 0; // seconds since start
      this.playing = false;
      this.speed = 1;
      this._rafLast = 0;
      this._pollTimer = null;
      this._wire();
    }

    _wire() {
      document.getElementById("pb-file").addEventListener("change", (e) => {
        const f = e.target.files[0];
        if (f) this.upload(f);
      });
      document.getElementById("pb-refresh").addEventListener("click", () => this.loadList());
      document.getElementById("pb-select").addEventListener("change", (e) => {
        const id = e.target.value;
        document.getElementById("pb-delete").disabled = !id;
        if (id) this.loadBag(id);
      });
      document.getElementById("pb-delete").addEventListener("click", () => this.deleteSelected());
      document.getElementById("pb-play").addEventListener("click", () => this.play());
      document.getElementById("pb-pause").addEventListener("click", () => this.pause());
      document.getElementById("pb-stop").addEventListener("click", () => this.stop());
      document.getElementById("pb-timeline").addEventListener("input", (e) => {
        if (!this.data) return;
        this.pause();
        this.t = (e.target.value / 1000) * this.data.duration;
        this._render();
      });
      document.getElementById("pb-speed").addEventListener("change", (e) => {
        this.speed = parseFloat(e.target.value);
      });

      this._tick = this._tick.bind(this);
      requestAnimationFrame(this._tick);
    }

    onEnter() {
      this.loadList();
    }
    onExit() {
      this.pause();
      clearTimeout(this._pollTimer);
    }

    async loadList() {
      try {
        const res = await Utils.api("/api/rosbags");
        const sel = document.getElementById("pb-select");
        const prev = sel.value;
        sel.innerHTML = '<option value="">— select rosbag —</option>';
        for (const b of res.rosbags) {
          const opt = document.createElement("option");
          opt.value = b.id;
          const size = Utils.formatBytes(b.size);
          opt.textContent = `${b.filename} (${size})`;
          sel.appendChild(opt);
        }
        if (prev) sel.value = prev;
      } catch (e) {
        window.App.toast("Could not list rosbags: " + e.message, "err");
      }
    }

    async upload(file) {
      const maxMb = this.settings.maxUploadMb;
      if (file.size > maxMb * 1024 * 1024) {
        window.App.toast(`File is ${Utils.formatBytes(file.size)}, over the ${maxMb} MB limit.`, "err");
        return;
      }
      const fd = new FormData();
      fd.append("file", file);
      this._showProgress(true, "Uploading…", 0);
      try {
        const res = await Utils.api("/api/upload-rosbag", { method: "POST", body: fd });
        window.App.toast("Uploaded. Extracting…", "ok");
        await this.loadList();
        document.getElementById("pb-select").value = res.id;
        document.getElementById("pb-delete").disabled = false;
        this._pollAndLoad(res.id);
      } catch (e) {
        this._showProgress(false);
        window.App.toast("Upload failed: " + e.message, "err");
      }
    }

    async loadBag(id) {
      this.stop();
      this._showProgress(true, "Extracting…", 0);
      this._pollAndLoad(id);
    }

    _pollAndLoad(id) {
      clearTimeout(this._pollTimer);
      const poll = async () => {
        try {
          const st = await Utils.api(`/api/rosbag/${id}/status`);
          if (st.status === "ready") {
            this._showProgress(false);
            await this._fetchOdometry(id);
          } else if (st.status === "error") {
            this._showProgress(false);
            window.App.toast("Extraction failed: " + (st.error || "unknown error"), "err");
          } else {
            this._showProgress(true, "Extracting…", st.progress || 0);
            this._pollTimer = setTimeout(poll, POLL_INTERVAL_MS);
          }
        } catch (e) {
          this._showProgress(false);
          window.App.toast("Status check failed: " + e.message, "err");
        }
      };
      poll();
    }

    async _fetchOdometry(id) {
      try {
        const res = await Utils.api(`/api/rosbag/${id}/odometry`);
        const points = res.odometry.map((p, i) => ({ ...p, index: i }));
        const meta = res.metadata;
        this.currentBagId = id;
        this.data = {
          points,
          startTime: meta.start,
          endTime: meta.end,
          duration: meta.duration || 0,
        };
        this.t = 0;
        this.renderer.setPath(points);
        this.renderer.fitToBounds();
        this._enableTransport(true);
        this._render();
        this._showMeta(meta, res.odometry.length);
      } catch (e) {
        window.App.toast("Could not load odometry: " + e.message, "err");
      }
    }

    async deleteSelected() {
      const id = document.getElementById("pb-select").value;
      if (!id) return;
      if (!confirm("Delete this rosbag and its cached extraction?")) return;
      try {
        await Utils.api(`/api/rosbag/${id}`, { method: "DELETE" });
        if (id === this.currentBagId) {
          this.data = null;
          this.renderer.clear();
          this._enableTransport(false);
        }
        await this.loadList();
        window.App.toast("Deleted", "ok");
      } catch (e) {
        window.App.toast("Delete failed: " + e.message, "err");
      }
    }

    /* -- transport --------------------------------------------------- */
    play() {
      if (!this.data) return;
      if (this.t >= this.data.duration) this.t = 0;
      this.playing = true;
      this._rafLast = performance.now();
      document.getElementById("pb-play").classList.add("hidden");
      document.getElementById("pb-pause").classList.remove("hidden");
    }
    pause() {
      this.playing = false;
      document.getElementById("pb-pause").classList.add("hidden");
      document.getElementById("pb-play").classList.remove("hidden");
    }
    stop() {
      this.pause();
      this.t = 0;
      if (this.data) this._render();
    }

    _tick(now) {
      if (this.playing && this.data) {
        const dt = (now - this._rafLast) / 1000;
        this._rafLast = now;
        this.t += dt * this.speed;
        if (this.t >= this.data.duration) {
          this.t = this.data.duration;
          this.pause();
        }
        this._render();
      } else {
        this._rafLast = now;
      }
      requestAnimationFrame(this._tick);
    }

    _render() {
      if (!this.data) return;
      const pose = this._interpolate(this.t);
      const idx = this._indexAtTime(this.t);
      this.renderer.setCurrent(pose, idx);
      this._updateTimeUi();
    }

    _updateTimeUi() {
      const d = this.data.duration || 1;
      const frac = Utils.clamp(this.t / d, 0, 1);
      document.getElementById("pb-timeline").value = Math.round(frac * 1000);
      document.getElementById("pb-time").textContent =
        `${Utils.formatTime(this.t)} / ${Utils.formatTime(this.data.duration)}`;
    }

    /** Index of the last point at or before relative time t. */
    _indexAtTime(t) {
      const pts = this.data.points;
      const target = this.data.startTime + t;
      // Linear scan is fine for typical bags; binary search if this grows.
      let lo = 0, hi = pts.length - 1, ans = 0;
      while (lo <= hi) {
        const mid = (lo + hi) >> 1;
        if (pts[mid].timestamp <= target) {
          ans = mid;
          lo = mid + 1;
        } else {
          hi = mid - 1;
        }
      }
      return ans;
    }

    _interpolate(t) {
      const pts = this.data.points;
      if (pts.length === 1) return pts[0];
      const target = this.data.startTime + t;
      const i = this._indexAtTime(t);
      if (i >= pts.length - 1) return pts[pts.length - 1];
      const a = pts[i];
      const b = pts[i + 1];
      const span = b.timestamp - a.timestamp;
      const f = span > 0 ? Utils.clamp((target - a.timestamp) / span, 0, 1) : 0;
      return {
        x: a.x + (b.x - a.x) * f,
        y: a.y + (b.y - a.y) * f,
        heading: Utils.lerpAngle(a.heading, b.heading, f),
      };
    }

    /* -- ui helpers -------------------------------------------------- */
    _enableTransport(on) {
      document.getElementById("pb-play").disabled = !on;
      document.getElementById("pb-stop").disabled = !on;
      document.getElementById("pb-timeline").disabled = !on;
    }

    _showProgress(show, label, pct) {
      const wrap = document.getElementById("pb-progress");
      if (!show) {
        wrap.classList.add("hidden");
        return;
      }
      wrap.classList.remove("hidden");
      document.getElementById("pb-progress-bar").style.width = `${pct || 0}%`;
      document.getElementById("pb-progress-label").textContent =
        `${label} ${pct ? pct + "%" : ""}`.trim();
    }

    _showMeta(meta, count) {
      document.getElementById("pb-meta").textContent =
        `Duration ${Utils.formatTime(meta.duration)} · ${count} points`;
    }
  }

  window.PlaybackMode = PlaybackMode;
})();
