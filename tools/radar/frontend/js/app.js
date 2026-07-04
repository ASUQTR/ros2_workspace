(function () {
  const wsStatus = document.getElementById("ws-status");
  const scanForm = document.getElementById("scan-form");
  const localizeForm = document.getElementById("localize-form");
  const scanStatusEl = document.getElementById("scan-status");
  const localizeStatusEl = document.getElementById("localize-status");

  function connect() {
    const proto = window.location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(`${proto}://${window.location.host}/ws`);

    ws.onopen = () => {
      wsStatus.textContent = "connected";
      wsStatus.className = "status status-connected";
    };
    ws.onclose = () => {
      wsStatus.textContent = "disconnected — retrying…";
      wsStatus.className = "status status-disconnected";
      setTimeout(connect, 2000);
    };
    ws.onerror = () => ws.close();

    ws.onmessage = (event) => {
      const frame = JSON.parse(event.data);
      if (frame.type === "scan") {
        window.SonarView.render(frame.data);
      } else if (frame.type === "localization") {
        window.PoolView.render(frame.data);
      }
    };
  }

  scanForm.addEventListener("submit", async (event) => {
    event.preventDefault();
    scanStatusEl.textContent = "scanning…";
    const body = {
      start_angle: Number(document.getElementById("scan-start").value),
      stop_angle: Number(document.getElementById("scan-stop").value),
      desired_range: Number(document.getElementById("scan-range").value),
    };
    try {
      const res = await fetch("/api/scan", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const result = await res.json();
      scanStatusEl.textContent = result.success ? "ok" : `failed: ${result.message}`;
    } catch (err) {
      scanStatusEl.textContent = `error: ${err}`;
    }
  });

  localizeForm.addEventListener("submit", async (event) => {
    event.preventDefault();
    localizeStatusEl.textContent = "localizing…";
    try {
      const res = await fetch("/api/localize", { method: "POST" });
      const result = await res.json();
      localizeStatusEl.textContent = result.success ? "ok" : `failed: ${result.message}`;
    } catch (err) {
      localizeStatusEl.textContent = `error: ${err}`;
    }
  });

  connect();
})();
