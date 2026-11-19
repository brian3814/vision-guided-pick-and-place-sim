/**
 * The page's one link to Kit, as React state: the stream's status, the cell's state, and the
 * events. Once the stream is live the page says hello, and Kit answers with every event of
 * the session so far, which replace what the page had: a reload shows the results as they were.
 */
import { useCallback, useEffect, useRef, useState } from 'react';
import { IGNORE, KitLink } from './link';
import type { LinkStatus } from './link';
import { PROTOCOL } from './protocol';
import type { Action, Answer, CellEvent, CellState } from './protocol';
import { nvidiaStreamer } from './streamer';

const HELLO_WAIT_MS = 2000; // for each answer
const HELLO_PAUSE_MS = 1000; // between tries
export const WAITING = 'waiting for the cell: Kit answers once it is built';

let shared: KitLink | null = null;

/** One stream per page, kept across React's remounts and hot reloads. */
export function sharedLink(): KitLink {
    if (shared === null) shared = new KitLink(nvidiaStreamer());
    return shared;
}

export interface Kit {
    link: KitLink;
    status: LinkStatus;
    info: string;
    state: CellState | null;
    request(action: Action, args?: Record<string, unknown>): Promise<Answer>;
}

/** `onEvents(events, replace)`: events to add to the results, or with `replace` all of them (hello's log). */
export type OnEvents = (events: CellEvent[], replace: boolean) => void;

export function useKit(onEvents: OnEvents): Kit {
    const link = sharedLink();
    const [status, setStatus] = useState<LinkStatus>(link.status);
    const [info, setInfo] = useState(link.info);
    const [state, setState] = useState<CellState | null>(null);
    const handler = useRef(onEvents);
    handler.current = onEvents;

    useEffect(() => {
        // The stream reports itself started before its data channel is open: a message sent then is refused
        // at once, or lost on the way. And Kit answers only once the cell is built, which can take minutes after
        // the stream starts. So hello is asked again, a moment later each time, for as long as the stream is live.
        let cancelled = false;
        const hello = (tries = 0) => {
            if (cancelled) return;
            link.request('hello', {}, HELLO_WAIT_MS)
                .then((answer) => {
                    if (answer.protocol !== PROTOCOL) {
                        setInfo(`Kit speaks protocol ${answer.protocol}; this page speaks ${PROTOCOL}`);
                        return;
                    }
                    setInfo('streaming');
                    handler.current(answer.log ?? [], true);
                })
                .catch((error: Error) => {
                    if (link.status !== 'live') {
                        setInfo(error.message);
                        return;
                    }
                    if (tries === 2) setInfo(WAITING);
                    setTimeout(() => hello(tries + 1), HELLO_PAUSE_MS);
                });
        };
        link.listener = {
            event: (event) => handler.current([event], false),
            state: setState,
            status: (next, why) => {
                setStatus(next);
                setInfo(why);
                if (next === 'live') hello();
            },
        };
        // The viewport, a child, may have started the stream before this listener was there.
        setStatus(link.status);
        setInfo(link.info);
        if (link.status === 'live') hello(); // remounted on a live stream
        return () => {
            cancelled = true;
            link.listener = IGNORE;
        };
    }, [link]);

    const request = useCallback((action: Action, args?: Record<string, unknown>) => link.request(action, args), [link]);
    return { link, status, info, state, request };
}
