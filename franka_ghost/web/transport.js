// [THROWAWAY] Session C 20 Hz polling seam; delete when merged into franka_web.

const STATE_SCHEMA = "franka.ghost.state/1";

/** Wire the throwaway loopback state endpoint to the durable by-name scene API. */
export function startStatePolling(handle, {
  stateUrl = "./state.json",
  rateHz = 20,
  onState = () => {},
  onError = () => {},
} = {}) {
  if (!handle || typeof handle.setMeasured !== "function") {
    throw new TypeError("handle must provide setMeasured(map)");
  }
  if (!Number.isFinite(rateHz) || rateHz <= 0) {
    throw new TypeError("rateHz must be positive and finite");
  }
  const periodMs = 1000 / rateHz;
  let stopped = false;
  let timer = null;
  let controller = null;

  async function poll() {
    if (stopped) {
      return;
    }
    controller = new AbortController();
    try {
      const response = await fetch(stateUrl, {cache: "no-store", signal: controller.signal});
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }
      const state = await response.json();
      if (!state || state.schema !== STATE_SCHEMA || !state.joints
          || typeof state.joints !== "object") {
        throw new Error("invalid state frame");
      }
      if (!state.stale) {
        handle.setMeasured(state.joints);
      }
      onState(state);
    } catch (error) {
      if (!stopped && error.name !== "AbortError") {
        onError(error);
      }
    } finally {
      controller = null;
      if (!stopped) {
        timer = setTimeout(poll, periodMs);
      }
    }
  }

  poll();
  return {
    stop() {
      if (stopped) {
        return;
      }
      stopped = true;
      clearTimeout(timer);
      if (controller) {
        controller.abort();
      }
    },
  };
}

/** POST one frozen C1 event to the throwaway validation-only endpoint. */
export async function postApply(event, {applyUrl = "./apply"} = {}) {
  const response = await fetch(applyUrl, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(event),
  });
  if (!response.ok) {
    throw new Error(`Apply endpoint returned HTTP ${response.status}`);
  }
  const verdict = await response.json();
  if (!verdict || typeof verdict.accepted !== "boolean") {
    throw new Error("Apply endpoint returned an invalid verdict");
  }
  return verdict;
}
