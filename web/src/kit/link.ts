/**
 * The page's connection to Kit: the stream, the requests the page sends and the answers they
 * get, and the events and state Kit sends on its own (see protocol.ts).
 */
import { EVENT, REQUEST, RESPONSE, STATE } from './protocol';
import type { Action, Answer, CellEvent, CellState } from './protocol';
import type { StreamOptions, Streamer } from './streamer';

export type LinkStatus = 'idle' | 'connecting' | 'live' | 'failed';

export interface LinkListener {
    event(event: CellEvent): void;
    state(state: CellState): void;
    status(status: LinkStatus, info: string): void;
}

interface Pending {
    resolve(answer: Answer): void;
    reject(error: Error): void;
    timer: ReturnType<typeof setTimeout>;
}

export const IGNORE: LinkListener = { event() {}, state() {}, status() {} };

function newId(): string {
    // randomUUID needs a secure context: localhost is one, a LAN address over http is not.
    if (typeof crypto !== 'undefined' && typeof crypto.randomUUID === 'function') return crypto.randomUUID();
    return `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
}

export class KitLink {
    status: LinkStatus = 'idle';
    info = '';
    listener: LinkListener = IGNORE;
    private pending = new Map<string, Pending>();

    constructor(
        private streamer: Streamer,
        private timeoutMs = 15000,
        private id: () => string = newId,
    ) {}

    /** Start the stream, once: React may mount the viewport twice in development. */
    connect(options: StreamOptions): void {
        if (this.status === 'connecting' || this.status === 'live') return;
        this.setStatus('connecting', `connecting to ${options.server}:${options.port}`);
        this.streamer.start(options, {
            started: () => this.setStatus('live', 'streaming'),
            failed: (why) => this.setStatus('failed', why),
            stopped: (why) => this.setStatus('failed', `the stream stopped: ${why}`),
            message: (message) => this.receive(message),
        });
    }

    /** Ask Kit to run `action`: resolves with its answer, rejects when it cannot be asked or does not answer
     * within `timeoutMs`. */
    request(action: Action, args: Record<string, unknown> = {}, timeoutMs = this.timeoutMs): Promise<Answer> {
        if (this.status !== 'live') return Promise.reject(new Error('not connected to Kit'));
        const id = this.id();
        return new Promise<Answer>((resolve, reject) => {
            const timer = setTimeout(() => {
                this.pending.delete(id);
                reject(new Error(`Kit did not answer "${action}"`));
            }, timeoutMs);
            this.pending.set(id, { resolve, reject, timer });
            // The id goes in the payload too: the library does not send the message's own id, and Kit answers
            // with `request_id` (see web_bridge.py).
            this.streamer.send({ event_type: REQUEST, id, payload: { action, request_id: id, ...args } }).catch((error: unknown) => {
                this.settle(id);
                reject(new Error(`could not send "${action}": ${String(error)}`));
            });
        });
    }

    /** A custom message from the stream. Kit's messages are `{event_type, id, payload}`; others are the stream's. */
    receive(message: unknown): void {
        if (!message || typeof message !== 'object' || !('event_type' in message)) return;
        const { event_type: type, payload } = message as { event_type: string; payload?: unknown };
        if (!payload || typeof payload !== 'object') return;
        const body = payload as Record<string, unknown>;
        if (type === RESPONSE) {
            const pending = this.settle(String(body.request_id));
            if (!pending) return;
            const answer = body as Answer;
            if (answer.state) this.listener.state(answer.state);
            pending.resolve(answer);
        } else if (type === EVENT && body.event) {
            this.listener.event(body.event as CellEvent);
        } else if (type === STATE && body.state) {
            this.listener.state(body.state as CellState);
        }
    }

    private settle(id: string): Pending | undefined {
        const pending = this.pending.get(id);
        if (pending) {
            clearTimeout(pending.timer);
            this.pending.delete(id);
        }
        return pending;
    }

    private setStatus(status: LinkStatus, info: string): void {
        this.status = status;
        this.info = info;
        if (status === 'failed') {
            for (const [id, pending] of this.pending) {
                this.settle(id);
                pending.reject(new Error(info));
            }
        }
        this.listener.status(status, info);
    }
}
