/**
 * Kit's viewport, streamed: the video the stream draws into, and what to say while there is
 * none. The stream sends the mouse and keyboard back to Kit while the video has focus, so the
 * viewport is used as in Isaac Sim: click it, then orbit, zoom, select.
 */
import { useEffect, useLayoutEffect, useRef } from 'react';
import type { RefObject } from 'react';
import type { KitLink, LinkStatus } from './link';
import type { StreamOptions } from './streamer';

// In index.html, outside React.
export const VIDEO_ID = 'remote-video';
export const AUDIO_ID = 'remote-audio';
const PARKING_ID = 'stream-parking';
const SIGNAL_PORT = 49100; // omni.kit.livestream.app's primaryStream.signalPort

/** Where Kit is: this page's host unless `?server=` says otherwise, on `?port=` or 49100. */
export function streamOptions(location: Pick<Location, 'search' | 'hostname'>): StreamOptions {
    const query = new URLSearchParams(location.search);
    const server = query.get('server');
    return {
        server: server || location.hostname || '127.0.0.1',
        ...(server ? { mediaServer: server } : {}),
        port: Number(query.get('port')) || SIGNAL_PORT,
        videoElementId: VIDEO_ID,
        audioElementId: AUDIO_ID,
        width: 1920,
        height: 1080,
    };
}

const PROBE_PAUSE_MS = 2000;

/**
 * Until Kit's signalling server answers. Kit starts it a while after it is launched, and the streaming
 * library gives up after a few tries, so the page waits for it first: any answer will do (the server
 * speaks WebSocket), and a refused connection rejects the fetch.
 */
async function untilListening(options: StreamOptions, cancelled: () => boolean): Promise<void> {
    while (!cancelled()) {
        try {
            await fetch(`http://${options.server}:${options.port}/`, { mode: 'no-cors', cache: 'no-store' });
            return;
        } catch {
            await new Promise((resolve) => setTimeout(resolve, PROBE_PAUSE_MS));
        }
    }
}

interface Props {
    link: KitLink;
    status: LinkStatus;
    info: string;
}

/**
 * Let the video have the keyboard only when the user clicks it. The streaming library focuses it
 * itself once the stream starts, and whatever is typed then (a start pose being typed in the
 * panel) would reach Kit as shortcuts (F frames the selection). Focus it takes that way goes
 * back, once the focus event is over, to what had it.
 */
function useFocusOnlyByPointer(id: string): void {
    useEffect(() => {
        const video = document.getElementById(id);
        if (!video) return;
        let pressed = false;
        let previous: HTMLElement | null = null; // the last element focused other than the video
        const press = () => {
            pressed = true;
        };
        const remember = (event: FocusEvent) => {
            if (event.target !== video && event.target instanceof HTMLElement) previous = event.target;
        };
        const focus = (event: FocusEvent) => {
            if (pressed) return;
            const back = event.relatedTarget instanceof HTMLElement ? event.relatedTarget : previous;
            setTimeout(() => {
                if (document.activeElement !== video || pressed) return;
                if (back && back.isConnected) back.focus();
                else video.blur();
            }, 0);
        };
        const blur = () => {
            pressed = false;
        };
        video.addEventListener('pointerdown', press, true);
        video.addEventListener('focus', focus);
        video.addEventListener('blur', blur);
        document.addEventListener('focusin', remember);
        return () => {
            video.removeEventListener('pointerdown', press, true);
            video.removeEventListener('focus', focus);
            video.removeEventListener('blur', blur);
            document.removeEventListener('focusin', remember);
        };
    }, [id]);
}

/**
 * Show the stream's video, from index.html, in `frame` while mounted. On unmount it goes back to
 * its hidden parking place, still in the document, so it keeps playing what the library gives it.
 */
function useParkedVideo(frame: RefObject<HTMLDivElement | null>): void {
    useLayoutEffect(() => {
        const video = document.getElementById(VIDEO_ID);
        const parking = document.getElementById(PARKING_ID);
        if (!video || !frame.current) return;
        frame.current.appendChild(video);
        return () => {
            parking?.appendChild(video);
        };
    }, [frame]);
}

export function Viewport({ link, status, info }: Props) {
    const frame = useRef<HTMLDivElement>(null);
    useParkedVideo(frame);
    useEffect(() => {
        let cancelled = false;
        const options = streamOptions(window.location);
        untilListening(options, () => cancelled).then(() => {
            if (!cancelled) link.connect(options);
        });
        return () => {
            cancelled = true;
        };
    }, [link]);
    useFocusOnlyByPointer(VIDEO_ID);

    return (
        <div className="viewport">
            <div className="video-frame" ref={frame} />
            {status !== 'live' && (
                <div className="viewport-overlay">
                    <p>{status === 'failed' ? 'No stream from Kit.' : 'Waiting for the stream from Kit…'}</p>
                    <p className="dim">{status === 'idle' ? 'Start it with cell_app.py --web; the page connects once it is up.' : info}</p>
                    {status === 'failed' && (
                        <button type="button" onClick={() => window.location.reload()}>
                            Reconnect
                        </button>
                    )}
                </div>
            )}
        </div>
    );
}
