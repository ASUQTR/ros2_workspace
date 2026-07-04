// Layer 2: renders SonarPositionResult messages (mirror of
// get_sonar_position's response) as a top-down pool + AUV marker. Pool
// dimensions come from the message itself (pool_width_x/pool_length_y),
// not a local constant, since only sonar_localization knows them.
window.PoolView = (function () {
  const canvas = document.getElementById("pool-canvas");
  const ctx = canvas.getContext("2d");
  const statusEl = document.getElementById("localize-status");
  const PADDING = 20;

  function clear() {
    ctx.fillStyle = "#001a1a";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
  }

  function quaternionToYaw(q) {
    return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
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

    const poolWidthX = result.pool_width_x;
    const poolLengthY = result.pool_length_y;
    if (!poolWidthX || !poolLengthY) return;

    const availableW = canvas.width - PADDING * 2;
    const availableH = canvas.height - PADDING * 2;
    const scale = Math.min(availableW / poolWidthX, availableH / poolLengthY);

    const originX = PADDING;
    const originY = canvas.height - PADDING;

    function toScreen(x, y) {
      return [originX + x * scale, originY - y * scale];
    }

    // Pool rectangle spans (0,0) SW corner to (poolWidthX, poolLengthY) NE corner.
    const [x0, y0] = toScreen(0, 0);
    const [x1, y1] = toScreen(poolWidthX, poolLengthY);
    ctx.strokeStyle = "#00ff80";
    ctx.lineWidth = 2;
    ctx.strokeRect(x0, y1, x1 - x0, y0 - y1);

    const pose = result.pose?.pose?.pose;
    if (!pose) return;

    const [px, py] = toScreen(pose.position.x, pose.position.y);
    const yaw = quaternionToYaw(pose.orientation);

    // Triangle marker pointing along the AUV's yaw (screen y grows downward,
    // pool y grows "up", hence the sign flip on rotation).
    const size = 10;
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-yaw);
    ctx.fillStyle = "#ffcc00";
    ctx.beginPath();
    ctx.moveTo(size, 0);
    ctx.lineTo(-size * 0.6, size * 0.6);
    ctx.lineTo(-size * 0.6, -size * 0.6);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  return { render };
})();
