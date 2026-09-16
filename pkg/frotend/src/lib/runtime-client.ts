import { SceneConfig, SimulationSnapshot } from '../types/simulation';

type SnapshotListener = (snapshot: SimulationSnapshot) => void;
type ConnectionListener = (status: 'connected' | 'connecting' | 'disconnected') => void;

class RuntimeClient {
  private socket: WebSocket | null = null;
  private snapshotListeners = new Set<SnapshotListener>();
  private connectionListeners = new Set<ConnectionListener>();

  connect(): void {
    if (this.socket && (this.socket.readyState === WebSocket.OPEN || this.socket.readyState === WebSocket.CONNECTING)) return;
    this.notifyConnection('connecting');
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const socket = new WebSocket(`${protocol}//${window.location.host}/ws`);
    this.socket = socket;
    socket.onopen = () => {
      if (this.socket === socket) this.notifyConnection('connected');
    };
    socket.onmessage = (event) => {
      if (this.socket !== socket) return;
      const snapshot = JSON.parse(event.data) as SimulationSnapshot;
      this.snapshotListeners.forEach((listener) => listener(snapshot));
    };
    socket.onclose = () => {
      // A delayed close from React Strict Mode's first mount must not clear a
      // newer active socket created by the second mount.
      if (this.socket !== socket) return;
      this.socket = null;
      this.notifyConnection('disconnected');
    };
    socket.onerror = () => socket.close();
  }

  disconnect(): void {
    const socket = this.socket;
    this.socket = null;
    socket?.close();
  }
  subscribe(listener: SnapshotListener): () => void { this.snapshotListeners.add(listener); return () => this.snapshotListeners.delete(listener); }
  onConnection(listener: ConnectionListener): () => void { this.connectionListeners.add(listener); return () => this.connectionListeners.delete(listener); }

  async command<T = void>(path: string, body?: unknown): Promise<T> {
    const response = await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: body === undefined ? undefined : JSON.stringify(body) });
    if (!response.ok) throw new Error(await errorMessage(response));
    return await response.json().catch(() => undefined) as T;
  }

  async getConfig(): Promise<SceneConfig> {
    const response = await fetch('/api/config');
    if (!response.ok) throw new Error(await errorMessage(response));
    return (await response.json() as { config: SceneConfig }).config;
  }

  private notifyConnection(status: 'connected' | 'connecting' | 'disconnected'): void { this.connectionListeners.forEach((listener) => listener(status)); }
}

async function errorMessage(response: Response): Promise<string> {
  const body = await response.json().catch(() => null) as { error?: { message?: string } } | null;
  return body?.error?.message || `Server responded with status ${response.status}`;
}

export const runtimeClient = new RuntimeClient();
