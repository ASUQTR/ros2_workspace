/* 2D top-down canvas renderer with zoom/pan, path, and a heading gizmo.
 *
 * World frame: +x East, +y North (ENU). Screen y is flipped so North is up.
 * Draws on a requestAnimationFrame loop but only when marked dirty. Large
 * paths (>DECIMATE_ABOVE points) are decimated when drawing the full track.
 */
(function () {
  "use strict";

  const DECIMATE_ABOVE = 10000;

  class CanvasRenderer {
    constructor(canvas) {
      this.canvas = canvas;
      this.ctx = canvas.getContext("2d");
      this.dpr = window.devicePixelRatio || 1;

      this.points = []; // [{x, y, heading, timestamp}]
      this.progressIndex = 0; // last "traveled" index for playback highlight
      this.current = null; // {x, y, heading}

      // Camera: world point at screen center + pixels-per-meter.
      this.cam = { cx: 0, cy: 0, scale: 40 };
      this._dirty = true;
      this._hasFit = false;

      this._resize = this._resize.bind(this);
      this._loop = this._loop.bind(this);
      window.addEventListener("resize", this._resize);
      this._installInteraction();
      this._resize();
      requestAnimationFrame(this._loop);
    }

    /* ------------------------------------------------------------------ */
    /* Public data API                                                     */
    /* ------------------------------------------------------------------ */
    setPath(points) {
      this.points = points || [];
      this.progressIndex = this.points.length - 1;
      this.current = this.points.length ? this.points[this.points.length - 1] : null;
      this._dirty = true;
    }

    appendPoint(p) {
      this.points.push(p);
      this.progressIndex = this.points.length - 1;
      this.current = p;
      if (!this._hasFit && this.points.length >= 2) this.fitToBounds();
      this._dirty = true;
    }

    setCurrent(pose, progressIndex) {
      this.current = pose;
      if (typeof progressIndex === "number") this.progressIndex = progressIndex;
      this._dirty = true;
    }

    clear() {
      this.points = [];
      this.current = null;
      this.progressIndex = 0;
      this._hasFit = false;
      this._dirty = true;
    }

    requestRender() {
      this._dirty = true;
    }

    /* ------------------------------------------------------------------ */
    /* Camera                                                              */
    /* ------------------------------------------------------------------ */
    fitToBounds() {
      if (!this.points.length) return;
      let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
      for (const p of this.points) {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y;
        if (p.y > maxY) maxY = p.y;
      }
      this.cam.cx = (minX + maxX) / 2;
      this.cam.cy = (minY + maxY) / 2;
      const w = this.canvas.clientWidth || 1;
      const h = this.canvas.clientHeight || 1;
      const spanX = Math.max(maxX - minX, 1e-3);
      const spanY = Math.max(maxY - minY, 1e-3);
      const pad = 0.85;
      this.cam.scale = Math.min((w * pad) / spanX, (h * pad) / spanY);
      this.cam.scale = Utils.clamp(this.cam.scale, 0.5, 5000);
      this._hasFit = true;
      this._dirty = true;
    }

    worldToScreen(x, y) {
      const w = this.canvas.clientWidth;
      const h = this.canvas.clientHeight;
      return {
        sx: (x - this.cam.cx) * this.cam.scale + w / 2,
        sy: h / 2 - (y - this.cam.cy) * this.cam.scale,
      };
    }

    screenToWorld(sx, sy) {
      const w = this.canvas.clientWidth;
      const h = this.canvas.clientHeight;
      return {
        x: (sx - w / 2) / this.cam.scale + this.cam.cx,
        y: (h / 2 - sy) / this.cam.scale + this.cam.cy,
      };
    }

    /* ------------------------------------------------------------------ */
    /* Interaction                                                         */
    /* ------------------------------------------------------------------ */
    _installInteraction() {
      const c = this.canvas;
      let dragging = false;
      let lastX = 0, lastY = 0;

      c.addEventListener("mousedown", (e) => {
        dragging = true;
        lastX = e.clientX;
        lastY = e.clientY;
      });
      window.addEventListener("mouseup", () => (dragging = false));
      window.addEventListener("mousemove", (e) => {
        if (!dragging) return;
        const dx = e.clientX - lastX;
        const dy = e.clientY - lastY;
        lastX = e.clientX;
        lastY = e.clientY;
        this.cam.cx -= dx / this.cam.scale;
        this.cam.cy += dy / this.cam.scale;
        this._dirty = true;
      });
      c.addEventListener(
        "wheel",
        (e) => {
          e.preventDefault();
          const rect = c.getBoundingClientRect();
          const mx = e.clientX - rect.left;
          const my = e.clientY - rect.top;
          const before = this.screenToWorld(mx, my);
          const factor = e.deltaY < 0 ? 1.1 : 1 / 1.1;
          this.cam.scale = Utils.clamp(this.cam.scale * factor, 0.5, 5000);
          const after = this.screenToWorld(mx, my);
          // Keep the point under the cursor stationary.
          this.cam.cx += before.x - after.x;
          this.cam.cy += before.y - after.y;
          this._dirty = true;
        },
        { passive: false }
      );
      c.addEventListener("dblclick", () => this.fitToBounds());
    }

    _resize() {
      const w = this.canvas.clientWidth;
      const h = this.canvas.clientHeight;
      this.dpr = window.devicePixelRatio || 1;
      this.canvas.width = Math.round(w * this.dpr);
      this.canvas.height = Math.round(h * this.dpr);
      this.ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
      this._dirty = true;
    }

    /* ------------------------------------------------------------------ */
    /* Render loop                                                         */
    /* ------------------------------------------------------------------ */
    _loop() {
      if (this._dirty) {
        this._dirty = false;
        this._draw();
      }
      requestAnimationFrame(this._loop);
    }

    _draw() {
      const ctx = this.ctx;
      const w = this.canvas.clientWidth;
      const h = this.canvas.clientHeight;
      ctx.clearRect(0, 0, w, h);
      this._drawGrid();
      this._drawPath();
      this._drawSubmarine();
      this._updateHud();
    }

    _drawGrid() {
      const ctx = this.ctx;
      const w = this.canvas.clientWidth;
      const h = this.canvas.clientHeight;

      // Pick a "nice" grid spacing in meters given the current zoom.
      const targetPx = 80;
      let step = Math.pow(10, Math.floor(Math.log10(targetPx / this.cam.scale)));
      const stepPx = step * this.cam.scale;
      if (stepPx < targetPx / 2) step *= 5;
      else if (stepPx < targetPx) step *= 2;

      const tl = this.screenToWorld(0, 0);
      const br = this.screenToWorld(w, h);
      ctx.lineWidth = 1;
      ctx.strokeStyle = "#1b2430";
      ctx.fillStyle = "#495260";
      ctx.font = "10px monospace";

      const startX = Math.floor(tl.x / step) * step;
      for (let x = startX; x <= br.x; x += step) {
        const s = this.worldToScreen(x, 0);
        ctx.beginPath();
        ctx.moveTo(s.sx, 0);
        ctx.lineTo(s.sx, h);
        ctx.stroke();
        ctx.fillText(`${x.toFixed(step < 1 ? 1 : 0)}`, s.sx + 2, h - 4);
      }
      const startY = Math.floor(br.y / step) * step;
      for (let y = startY; y <= tl.y; y += step) {
        const s = this.worldToScreen(0, y);
        ctx.beginPath();
        ctx.moveTo(0, s.sy);
        ctx.lineTo(w, s.sy);
        ctx.stroke();
        ctx.fillText(`${y.toFixed(step < 1 ? 1 : 0)}`, 2, s.sy - 2);
      }

      // Origin axes.
      const o = this.worldToScreen(0, 0);
      ctx.strokeStyle = "#3a4655";
      ctx.beginPath();
      ctx.moveTo(o.sx, 0);
      ctx.lineTo(o.sx, h);
      ctx.moveTo(0, o.sy);
      ctx.lineTo(w, o.sy);
      ctx.stroke();
    }

    _drawPath() {
      const n = this.points.length;
      if (n < 2) return;
      const ctx = this.ctx;
      const stride = n > DECIMATE_ABOVE ? Math.ceil(n / DECIMATE_ABOVE) : 1;

      // Faint full track.
      ctx.strokeStyle = "#30425c";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      let started = false;
      for (let i = 0; i < n; i += stride) {
        const s = this.worldToScreen(this.points[i].x, this.points[i].y);
        if (!started) {
          ctx.moveTo(s.sx, s.sy);
          started = true;
        } else {
          ctx.lineTo(s.sx, s.sy);
        }
      }
      ctx.stroke();

      // Bright traveled portion up to progressIndex.
      const limit = Utils.clamp(this.progressIndex, 0, n - 1);
      ctx.strokeStyle = "#58a6ff";
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      started = false;
      for (let i = 0; i <= limit; i += stride) {
        const s = this.worldToScreen(this.points[i].x, this.points[i].y);
        if (!started) {
          ctx.moveTo(s.sx, s.sy);
          started = true;
        } else {
          ctx.lineTo(s.sx, s.sy);
        }
      }
      // Ensure the exact current index is connected.
      const last = this.worldToScreen(this.points[limit].x, this.points[limit].y);
      ctx.lineTo(last.sx, last.sy);
      ctx.stroke();
    }

    _drawSubmarine() {
      if (!this.current) return;
      const ctx = this.ctx;
      const { sx, sy } = this.worldToScreen(this.current.x, this.current.y);
      const h = this.current.heading || 0;
      const size = 14;

      // Forward direction in screen space (y flipped).
      const fx = Math.cos(h);
      const fy = -Math.sin(h);
      const rx = -fy; // perpendicular
      const ry = fx;

      const tip = { x: sx + fx * size, y: sy + fy * size };
      const bl = { x: sx - fx * size * 0.6 + rx * size * 0.6, y: sy - fy * size * 0.6 + ry * size * 0.6 };
      const br = { x: sx - fx * size * 0.6 - rx * size * 0.6, y: sy - fy * size * 0.6 - ry * size * 0.6 };

      ctx.fillStyle = "#ffcc33";
      ctx.strokeStyle = "#0d1117";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(tip.x, tip.y);
      ctx.lineTo(bl.x, bl.y);
      ctx.lineTo(br.x, br.y);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    }

    _updateHud() {
      const hud = document.getElementById("hud");
      if (!hud) return;
      if (!this.current) {
        hud.textContent = "";
        return;
      }
      const deg = ((this.current.heading * 180) / Math.PI).toFixed(1);
      hud.textContent =
        `x: ${this.current.x.toFixed(2)} m   y: ${this.current.y.toFixed(2)} m\n` +
        `heading: ${deg}°   scale: ${this.cam.scale.toFixed(1)} px/m`;
    }
  }

  window.CanvasRenderer = CanvasRenderer;
})();
