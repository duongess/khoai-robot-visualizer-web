import { SceneUpdatePayload, SceneConfig } from '../types/simulation';

export interface ApiSceneResponse {
  success: boolean;
  config?: SceneConfig;
  error?: string;
}

/**
 * Sends scene updates to the same-origin Go backend.
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
        config: data.config,
      };
    } else {
      const errJson = await res.json().catch(() => null);
      const errorMsg = errJson?.error || `Server responded with status ${res.status}`;
      return { success: false, error: errorMsg };
    }
  } catch (error) {
    return { success: false, error: error instanceof Error ? error.message : 'Scene update failed.' };
  }
}
