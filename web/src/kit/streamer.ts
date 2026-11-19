/**
 * The one place that uses NVIDIA's WebRTC streaming library (@nvidia/ov-web-rtc, 6.x): a
 * direct stream to a Kit app on this machine or the network, its video in an element of the
 * page, and its custom messages. link.ts only knows the `Streamer` interface, so it is tested
 * without a browser.
 */
import { AppStreamer, EventAction, EventStatus, StreamType } from '@nvidia/ov-web-rtc';
import type { StreamEvent } from '@nvidia/ov-web-rtc';

export interface StreamOptions {
    /** The Kit app's host: its signalling server. */
    server: string;
    /** Where its media comes from, when it is not what Kit advertises. Left unset for a Kit app on this
     * machine: `localhost` can resolve to ::1, and Kit's media port listens on IPv4. */
    mediaServer?: string;
    /** Its signalling port: primaryStream.signalPort, 49100 by default. */
    port: number;
    videoElementId: string;
    audioElementId: string;
    /** The largest stream: Kit renders at the video element's size, up to this. */
    width: number;
    height: number;
}

export interface StreamCallbacks {
    started(): void;
    failed(why: string): void;
    stopped(why: string): void;
    message(message: unknown): void;
}

/** A message for the Kit app's messaging extension. */
export interface OutgoingMessage {
    event_type: string;
    id: string;
    payload: Record<string, unknown>;
}

export interface Streamer {
    start(options: StreamOptions, callbacks: StreamCallbacks): void;
    send(message: OutgoingMessage): Promise<unknown>;
}

function why(value: unknown): string {
    if (value instanceof Error) return value.message;
    if (value && typeof value === 'object' && 'info' in value) return why((value as StreamEvent).info);
    return String(value);
}

export function nvidiaStreamer(): Streamer {
    const app = new AppStreamer();
    return {
        start(options, callbacks) {
            app.connect({
                streamSource: StreamType.DIRECT,
                streamConfig: {
                    signalingServer: options.server,
                    signalingPort: options.port,
                    ...(options.mediaServer ? { mediaServer: options.mediaServer } : {}),
                    videoElementId: options.videoElementId,
                    audioElementId: options.audioElementId,
                    authenticate: false,
                    width: options.width,
                    height: options.height,
                    fps: 60,
                    maxReconnects: 5,
                    onStart: (event: StreamEvent) => {
                        if (event.action !== EventAction.START) return;
                        if (event.status === EventStatus.SUCCESS) callbacks.started();
                        else if (event.status === EventStatus.ERROR) callbacks.failed(why(event));
                    },
                    onStop: (event: StreamEvent) => callbacks.stopped(why(event)),
                    onTerminate: (event: StreamEvent) => callbacks.stopped(why(event)),
                    onCustomEvent: (message: unknown) => callbacks.message(message),
                },
            }).catch((error: unknown) => callbacks.failed(why(error)));
        },
        send: (message) => app.sendMessage(message),
    };
}
