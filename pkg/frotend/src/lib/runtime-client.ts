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
    this.socket = new WebSocket(`${protocol}//${window.location.host}/ws`);
    this.socket.onopen = () => this.notifyConnection('connected');
    this.socket.onmessage = (event) => {
      const snapshot = JSON.parse(event.data) as SimulationSnapshot;
      this.snapshotListeners.forEach((listener) => listener(snapshot));
    };
    this.socket.onclose = () => {
      this.socket = null;
      this.notifyConnection('disconnected');
    };
    this.socket.onerror = () => this.socket?.close();
  }

  disconnect(): void { this.socket?.close(); this.socket = null; }
  subscribe(listener: SnapshotListener): () => void { this.snapshotListeners.add(listener); return () => this.snapshotListeners.delete(listener); }
  onConnection(listener: ConnectionListener): () => void { this.connectionListeners.add(listener); return () => this.connectionListeners.delete(listener); }

  async command(path: string, body?: unknown): Promise<void> {
    const response = await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: body === undefined ? undefined : JSON.stringify(body) });
    if (!response.ok) throw new Error(await errorMessage(response));
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
