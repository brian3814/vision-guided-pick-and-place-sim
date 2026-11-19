import { describe, expect, it, vi } from 'vitest';
import { KitLink } from './link';
import type { LinkListener } from './link';
import type { CellEvent, CellState } from './protocol';
import type { OutgoingMessage, StreamCallbacks, StreamOptions, Streamer } from './streamer';

const OPTIONS: StreamOptions = {
    server: '127.0.0.1', port: 49100, videoElementId: 'v', audioElementId: 'a', width: 1920, height: 1080,
};

class FakeStreamer implements Streamer {
    starts = 0;
    sent: OutgoingMessage[] = [];
    callbacks!: StreamCallbacks;
    refuse = false;

    start(_options: StreamOptions, callbacks: StreamCallbacks): void {
        this.starts += 1;
        this.callbacks = callbacks;
    }

    send(message: OutgoingMessage): Promise<unknown> {
        this.sent.push(message);
        return this.refuse ? Promise.reject(new Error('closed')) : Promise.resolve();
    }
}

function setup() {
    const streamer = new FakeStreamer();
    let n = 0;
    const link = new KitLink(streamer, 1000, () => `id${++n}`);
    const seen = { events: [] as CellEvent[], states: [] as CellState[], statuses: [] as string[] };
    const listener: LinkListener = {
        event: (e) => seen.events.push(e),
        state: (s) => seen.states.push(s),
        status: (s) => seen.statuses.push(s),
    };
    link.listener = listener;
    return { streamer, link, seen };
}

const STATE = { busy: false } as unknown as CellState;

describe('KitLink', () => {
    it('starts the stream once, however often the viewport mounts', () => {
        const { streamer, link, seen } = setup();
        link.connect(OPTIONS);
        link.connect(OPTIONS);
        expect(streamer.starts).toBe(1);
        streamer.callbacks.started();
        link.connect(OPTIONS);
        expect(streamer.starts).toBe(1);
        expect(seen.statuses).toEqual(['connecting', 'live']);
    });

    it('sends a request with an id and resolves with the answer of that id', async () => {
        const { streamer, link, seen } = setup();
        link.connect(OPTIONS);
        streamer.callbacks.started();
        const answer = link.request('run', { n: 5 });
        expect(streamer.sent).toEqual([
            { event_type: 'cell:request', id: 'id1', payload: { action: 'run', request_id: 'id1', n: 5 } },
        ]);
        // Kit's messaging answers under an id of its own: the answer is matched by request_id.
        link.receive({ event_type: 'cell:response', id: 'id1', payload: { id: 7, request_id: 'other', ok: false } });
        link.receive({ event_type: 'cell:response', id: '7', payload: { id: 7, request_id: 'id1', ok: true, state: STATE } });
        await expect(answer).resolves.toMatchObject({ ok: true });
        expect(seen.states).toEqual([STATE]);
    });

    it('passes events and state on, and leaves the stream\'s own messages alone', () => {
        const { link, seen } = setup();
        const event = { type: 'warning', at: 5, text: 'the wrist camera had been moved' };
        link.receive({ event_type: 'cell:event', id: 'x', payload: { event } });
        link.receive({ event_type: 'cell:state', id: 'y', payload: { state: STATE } });
        link.receive({ type: 'config', value: {} });
        link.receive('nonsense');
        expect(seen.events).toEqual([event]);
        expect(seen.states).toEqual([STATE]);
    });

    it('refuses to ask before the stream is live', async () => {
        const { link } = setup();
        await expect(link.request('hello')).rejects.toThrow('not connected to Kit');
    });

    it('gives up on an answer that does not come, and on a request that cannot be sent', async () => {
        vi.useFakeTimers();
        const { streamer, link } = setup();
        link.connect(OPTIONS);
        streamer.callbacks.started();
        const late = link.request('hello');
        vi.advanceTimersByTime(1001);
        await expect(late).rejects.toThrow('Kit did not answer "hello"');
        const quick = link.request('hello', {}, 100);
        vi.advanceTimersByTime(101);
        await expect(quick).rejects.toThrow('Kit did not answer "hello"');
        vi.useRealTimers();
        streamer.refuse = true;
        await expect(link.request('stop')).rejects.toThrow('could not send "stop"');
    });

    it('fails what waits when the stream stops', async () => {
        const { streamer, link, seen } = setup();
        link.connect(OPTIONS);
        streamer.callbacks.started();
        const waiting = link.request('hello');
        streamer.callbacks.stopped('server disconnected');
        await expect(waiting).rejects.toThrow('the stream stopped: server disconnected');
        expect(seen.statuses.at(-1)).toBe('failed');
        link.connect(OPTIONS);
        expect(streamer.starts).toBe(2);
    });
});
