import { SceneUpdatePayload, SceneConfig } from '../types/simulation';
import { mockTelemetryClient } from './mock-telemetry-client';

export interface ApiSceneResponse {
  success: boolean;
  config?: SceneConfig;
  error?: string;
}

/**
 * Sends scene updates to PUT /api/scene with automatic fallback to mockTelemetryClient
 */
export async function updateSceneApi(payload: SceneUpdatePayload): Promise<ApiSceneResponse> {
  try {
    const res = await fetch('/api/scene', {
      method: 'PUT',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(payload),
    });

    if (res.ok) {
      const data = await res.json();
      return {
        success: true,
        config: data.config || mockTelemetryClient.getConfig(),
      };
    } else {
      const errJson = await res.json().catch(() => null);
      const errorMsg = errJson?.error || `Server responded with status ${res.status}`;
      return { success: false, error: errorMsg };
    }
  } catch (_fetchErr) {
    // Client-side fallback if server endpoint is not intercepted
    return mockTelemetryClient.applySceneUpdate(payload);
  }
}
