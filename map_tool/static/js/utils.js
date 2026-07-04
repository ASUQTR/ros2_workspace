/* Shared helpers: math, formatting, and API access. Exposed on window.Utils. */
(function () {
  "use strict";

  const TWO_PI = Math.PI * 2;

  /** Yaw (radians) from a quaternion, ZYX convention. Matches the backend. */
  function quaternionToYaw(x, y, z, w) {
    const sinyCosp = 2 * (w * z + x * y);
    const cosyCosp = 1 - 2 * (y * y + z * z);
    return Math.atan2(sinyCosp, cosyCosp);
  }

  /** Wrap an angle to (-pi, pi]. */
  function wrapAngle(a) {
    let r = ((a + Math.PI) % TWO_PI + TWO_PI) % TWO_PI;
    return r - Math.PI;
  }

  /** Shortest-path interpolation between two angles (radians). */
  function lerpAngle(a, b, t) {
    const d = wrapAngle(b - a);
    return wrapAngle(a + d * t);
  }

  function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  /** Seconds -> "MM:SS" (or "H:MM:SS" past an hour). */
  function formatTime(sec) {
    if (!isFinite(sec) || sec < 0) sec = 0;
    const total = Math.floor(sec);
    const h = Math.floor(total / 3600);
    const m = Math.floor((total % 3600) / 60);
    const s = total % 60;
    const mm = String(m).padStart(2, "0");
    const ss = String(s).padStart(2, "0");
    return h > 0 ? `${h}:${mm}:${ss}` : `${mm}:${ss}`;
  }

  function formatBytes(n) {
    if (!n) return "0 B";
    const units = ["B", "KB", "MB", "GB"];
    const i = Math.floor(Math.log(n) / Math.log(1024));
    return `${(n / Math.pow(1024, i)).toFixed(1)} ${units[i]}`;
  }

  function getApiKey() {
    return localStorage.getItem("mapTool.apiKey") || "";
  }

  /** fetch() wrapper that injects the API key header and parses JSON errors. */
  async function api(path, opts) {
    opts = opts || {};
    opts.headers = opts.headers || {};
    const key = getApiKey();
    if (key) opts.headers["X-API-Key"] = key;
    const res = await fetch(path, opts);
    if (!res.ok) {
      let detail = res.statusText;
      try {
        const body = await res.json();
        if (body && body.detail) detail = body.detail;
      } catch (e) {
        /* non-JSON error body */
      }
      const err = new Error(detail);
      err.status = res.status;
      throw err;
    }
    if (res.status === 204) return null;
    return res.json();
  }

  window.Utils = {
    TWO_PI,
    quaternionToYaw,
    wrapAngle,
    lerpAngle,
    clamp,
    formatTime,
    formatBytes,
    getApiKey,
    api,
  };
})();
