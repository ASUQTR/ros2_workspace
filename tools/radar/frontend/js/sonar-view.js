// Layer 1: renders SonarScanResult messages (mirror of /sonar/scan's
// response) as a polar sweep centered on the AUV (sonar_link frame).
window.SonarView = (function () {
  const canvas = document.getElementById("sonar-canvas");
  const ctx = canvas.getContext("2d");
  const statusEl = document.getElementById("scan-status");

  function clear() {
    ctx.fillStyle = "#001a1a";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
  }

  function drawGrid(maxRadiusPx, maxRange) {
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    ctx.strokeStyle = "rgba(0, 255, 128, 0.25)";
    ctx.lineWidth = 1;
    for (let ring = 1; ring <= 4; ring++) {
      ctx.beginPath();
      ctx.arc(cx, cy, (maxRadiusPx * ring) / 4, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.fillStyle = "rgba(0, 255, 128, 0.6)";
    ctx.font = "10px monospace";
    ctx.fillText(`${maxRange.toFixed(0)} m`, cx + 4, cy - maxRadiusPx + 10);
  }

  function render(result) {
    clear();

    if (!result) {
      if (statusEl) statusEl.textContent = "no data yet";
      return;
    }
    if (!result.success) {
      if (statusEl) statusEl.textContent = `failed: ${result.message}`;
      return;
    }
    if (statusEl) statusEl.textContent = result.message || "ok";

    const scan = result.data;
    const maxRange = scan.range_max || 1;
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    const maxRadiusPx = Math.min(cx, cy) - 12;

    drawGrid(maxRadiusPx, maxRange);

    const angleMin = scan.angle_min;
    const angleIncrement = scan.angle_increment;
    const ranges = scan.ranges || [];
    const intensities = scan.intensities || [];

    for (let i = 0; i < ranges.length; i++) {
      const distance = ranges[i]?.echoes?.[0];
      const intensity = intensities[i]?.echoes?.[0] ?? 0;
      if (distance === undefined || distance <= 0) continue;

      // Sonar 0 deg = East wall direction; draw it pointing "up" on screen.
      const angleRad = angleMin + i * angleIncrement;
      const screenAngle = -angleRad + Math.PI / 2;
      const radiusPx = Math.min(distance / maxRange, 1) * maxRadiusPx;

      const x = cx + radiusPx * Math.cos(screenAngle);
      const y = cy - radiusPx * Math.sin(screenAngle);

      const alpha = Math.min(intensity / 255, 1);
      ctx.fillStyle = `rgba(0, 255, 128, ${0.2 + alpha * 0.8})`;
      ctx.beginPath();
      ctx.arc(x, y, 2, 0, Math.PI * 2);
      ctx.fill();
    }

    // AUV is always at the origin of its own sonar frame.
    ctx.fillStyle = "#ffffff";
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fill();
  }

  return { render };
})();
