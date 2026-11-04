"use strict";
// External compute card (Jetlink/Jetson) and the shared model library.
// comma 4 has no eGPU path any more; the file/DOM ids keep their old names
// so the Jetlink card and the library API stay where they were.
import { renderLibrary } from "./model_library.js";

const POLL_INTERVAL_MS = 2000;
let pollTimer = null;
let lastStatus = null;

const FALLBACK_STRINGS = {
  host_ok: "Connected",
  host_warning: "High temperature",
  host_error: "Host error / connection lost",
  host_unknown: "Waiting for current host health",
  host_ip: "Host IP",
  host_no_ip: "Unavailable",
  host_not_connected: "Host not connected",
  host_unavailable: "Host connection unavailable",
  host_health_unavailable: "Host health unavailable",
};

function t(key) {
  return typeof getUIText === "function" ? getUIText(`egpu_model_${key}`, FALLBACK_STRINGS[key] || key) : (FALLBACK_STRINGS[key] || key);
}

function render(status = lastStatus) {
  const card = document.getElementById("egpuModelCard");
  if (!card) return;
  lastStatus = status;
  renderLibrary(status?.library);
  const host = status?.jetlink;
  if (!host) {
    card.hidden = true;
    return;
  }

  const stateEl = document.getElementById("egpuModelState");
  const detailEl = document.getElementById("egpuModelDetail");
  let hostEl = document.getElementById("jetlinkHealth");
  if (!hostEl) {
    hostEl = document.createElement("div");
    hostEl.id = "jetlinkHealth";
    hostEl.className = "egpu-model-card__detail";
    detailEl.after(hostEl);
  }
  const severity = ["ok", "warning", "error", "unknown"].includes(host.severity) ? host.severity : "unknown";
  hostEl.hidden = false;
  hostEl.textContent = `${host.label} · ${t(`host_${severity}`)} · ${t("host_ip")}: ${(host.addresses || []).join(", ") || t("host_no_ip")}`
    + (Number.isFinite(host.temp_c) ? ` · ${host.temp_c.toFixed(1)} °C` : "")
      + (host.reason ? ` · ${({
        "Host not connected": t("host_not_connected"),
        "Host connection unavailable": t("host_unavailable"),
        "Host health unavailable": t("host_health_unavailable"),
      })[host.reason] || host.reason}` : "");
  hostEl.style.color = severity === "error" ? "#ff7474" : severity === "warning" ? "#ffd166" : "";

  card.hidden = false;
  card.dataset.state = host.severity === "error" ? "error" : "compiled";
  card.classList.remove("is-running");
  document.getElementById("egpuModelTitle").textContent = `${host.label} · ${status.library?.models?.find(m => m.id === status.library.effective)?.name || "Jetson"}`;
  stateEl.textContent = t(`host_${host.severity || "unknown"}`);
  detailEl.textContent = "";
}

async function refresh() {
  try {
    render(await getJson("/api/egpu/model"));
  } catch (_error) {
    if (lastStatus?.jetlink) render({ ...lastStatus, jetlink: {
      ...lastStatus.jetlink, severity: "unknown", addresses: [], temp_c: null, reason: "", fresh: false,
    } });
  } finally {
    window.clearTimeout(pollTimer);
    pollTimer = window.setTimeout(refresh, POLL_INTERVAL_MS);
  }
}

function init() {
  if (!pollTimer) refresh();
}

const CarrotEgpuModel = Object.freeze({ init, refresh, render });
globalThis.CarrotEgpuModel = CarrotEgpuModel;

export { CarrotEgpuModel };
