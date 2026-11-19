/**
 * The page against what Kit's bridge really sends: contract.json is one session recorded from
 * pick_place/web_bridge.py and the real controller, with a fake cell (pick_place/tests/test_web_contract.py,
 * which also fails when the bridge's messages change). It is played through the page's link, as the
 * stream would hand it over, and every state, event and answer is checked against protocol.ts.
 */
import { describe, expect, it } from 'vitest';
import { KitLink } from '../kit/link';
import { PROTOCOL } from '../kit/protocol';
import type { Action, Answer, CellEvent, CellState, Choice, GripperChoice, HeadingEvent, ResultEvent, WarningEvent } from '../kit/protocol';
import type { OutgoingMessage, StreamCallbacks, StreamOptions, Streamer } from '../kit/streamer';
import contract from './contract.json';
import { EMPTY, look, reduce } from './results';

type Step = { send: { action: Action; request_id: string } & Record<string, unknown> } | { receive: { event_type: string; payload: unknown } };
const steps = contract as unknown as Step[];

// What each field holds. `satisfies` ties these to protocol.ts: a field missing here, or one protocol.ts does not
// have, fails the type-check (npm run build).
type Kind = 'string' | 'number' | 'boolean' | 'array' | 'object' | 'number|null' | 'string|null';

const STATE = {
    perception: 'string', info: 'string', gripper: 'string', object: 'string', pose: 'number', busy: 'boolean',
    stopping: 'boolean', fixed_gripper: 'boolean', status: 'string', grippers: 'array', objects: 'array',
} satisfies Record<keyof CellState, Kind>;
const GRIPPER = { name: 'string', text: 'string', usable: 'boolean' } satisfies Record<keyof GripperChoice, Kind>;
const CHOICE = { name: 'string', text: 'string' } satisfies Record<keyof Choice, Kind>;
const HEADING = { type: 'string', gripper: 'string', object: 'string', text: 'string', at: 'number' } satisfies Record<keyof HeadingEvent, Kind>;
const RESULT = {
    type: 'string', pose: 'number', ok: 'boolean', text: 'string', outcome: 'string', place_error_m: 'number|null',
    tilt_deg: 'number|null', est_centre_mm: 'number|null', est_rot_deg: 'number|null', grasp_index: 'number|null',
    detail: 'string|null', cycle_s: 'number|null', at: 'number',
} satisfies Record<keyof ResultEvent, Kind>;
const WARNING = { type: 'string', text: 'string', at: 'number' } satisfies Record<keyof WarningEvent, Kind>;
const ANSWER = {
    request_id: 'string', ok: 'boolean', error: 'string', state: 'object', protocol: 'number', log: 'array',
} satisfies Record<keyof Answer, Kind>;
const EVENTS: Record<CellEvent['type'], Record<string, Kind>> = { heading: HEADING, result: RESULT, warning: WARNING };

function kindOf(value: unknown): string {
    if (value === null) return 'null';
    if (Array.isArray(value)) return 'array';
    return typeof value;
}

/** Whether `value` has the fields of `fields` (all of them, unless `optional`), each of its kind, and no others. */
function problems(value: unknown, fields: Record<string, Kind>, optional = false): string[] {
    if (kindOf(value) !== 'object') return [`not an object: ${JSON.stringify(value)}`];
    const object = value as Record<string, unknown>;
    const found = [];
    for (const key of Object.keys(object)) {
        if (!(key in fields)) found.push(`unknown field ${key}`);
        else if (!fields[key].split('|').includes(kindOf(object[key]))) found.push(`${key} is ${kindOf(object[key])}, not ${fields[key]}`);
    }
    if (!optional) for (const key of Object.keys(fields)) if (!(key in object)) found.push(`missing ${key}`);
    return found;
}

function stateProblems(state: CellState): string[] {
    return [
        ...problems(state, STATE),
        ...state.grippers.flatMap((g) => problems(g, GRIPPER)),
        ...state.objects.flatMap((o) => problems(o, CHOICE)),
    ];
}

const OPTIONS: StreamOptions = { server: '127.0.0.1', port: 49100, videoElementId: 'v', audioElementId: 'a', width: 1920, height: 1080 };

class Recorder implements Streamer {
    sent: OutgoingMessage[] = [];
    callbacks!: StreamCallbacks;

    start(_options: StreamOptions, callbacks: StreamCallbacks): void {
        this.callbacks = callbacks;
    }

    send(message: OutgoingMessage): Promise<unknown> {
        this.sent.push(message);
        return Promise.resolve();
    }
}

/** The session through the page's link: what it sent, and the events, states and answers it passed on. */
function replay() {
    const streamer = new Recorder();
    const ids = steps.flatMap((step) => ('send' in step ? [step.send.request_id] : []));
    let n = 0;
    const link = new KitLink(streamer, 1000, () => ids[n++]);
    const events: CellEvent[] = [];
    const states: CellState[] = [];
    link.listener = { event: (e) => events.push(e), state: (s) => states.push(s), status() {} };
    link.connect(OPTIONS);
    streamer.callbacks.started();
    const answers: Promise<Answer>[] = [];
    for (const step of steps) {
        if ('send' in step) {
            const { action, request_id: _id, ...args } = step.send;
            answers.push(link.request(action, args));
        } else {
            // The messaging extension puts its own id on each message; the page does not use it.
            link.receive({ ...step.receive, id: 'kit' });
        }
    }
    return { streamer, events, states, answers: Promise.all(answers) };
}

describe('the page against what the bridge sends (contract.json)', () => {
    it('sends each request as the bridge reads it', () => {
        const { streamer } = replay();
        const sent = steps.flatMap((step) => ('send' in step ? [step.send] : []));
        expect(streamer.sent.map((m) => m.event_type)).toEqual(sent.map(() => 'cell:request'));
        expect(streamer.sent.map((m) => m.payload)).toEqual(sent);
    });

    it('gets the answer to each request, by its request_id, each an Answer', async () => {
        const answers = await replay().answers;
        const asked = steps.flatMap((step) => ('send' in step ? [step.send] : []));
        expect(answers.map((a) => a.request_id)).toEqual(asked.map((s) => s.request_id));
        for (const answer of answers) {
            expect(problems(answer, ANSWER, true)).toEqual([]);
            expect(typeof answer.ok).toBe('boolean');
            expect(stateProblems(answer.state!)).toEqual([]);
            if (answer.ok === false) expect(answer.error).toBeTruthy();
        }
        const hellos = answers.filter((_answer, i) => asked[i].action === 'hello');
        expect(hellos.map((a) => a.protocol)).toEqual([PROTOCOL, PROTOCOL]);
        expect(answers.filter((a) => !a.ok).map((a) => a.error)).toEqual([
            'busy: wait until the cycles run, or the cell being built, are done',
            "unknown action: 'fly'",
        ]);
    });

    it('passes on states that are CellStates', () => {
        const { states } = replay();
        expect(states.length).toBeGreaterThan(10);
        for (const state of states) expect(stateProblems(state)).toEqual([]);
        expect(states.some((s) => s.busy) && states.some((s) => s.stopping)).toBe(true);
        expect(states.at(-1)).toMatchObject({ gripper: 'onrobot_rg2', object: 'toy', pose: 9, busy: false, stopping: false });
    });

    it('passes on events that are CellEvents', () => {
        const { events } = replay();
        for (const event of events) expect(problems(event, EVENTS[event.type])).toEqual([]);
        expect(events.map((e) => e.type)).toEqual(['heading', 'warning', 'result', 'result', 'heading', 'result', 'heading', 'result']);
        expect(events.filter((e): e is ResultEvent => e.type === 'result').map((e) => e.est_centre_mm)).toEqual([0.6, 0.6, 0.6, null]);
    });

    it('draws the same results list from the events as they came and from hello\'s log', async () => {
        const { events, answers } = replay();
        const live = reduce(EMPTY, { type: 'add', events });
        const log = (await answers).at(-1)!.log!;
        const reloaded = reduce(EMPTY, { type: 'replace', events: log });
        expect(reloaded.rows.map((r) => r.event)).toEqual(live.rows.map((r) => r.event));
        expect(live.rows.map((r) => look(r.event))).toEqual(['heading', 'bad', 'good', 'bad', 'heading', 'good', 'heading', 'good']);
        expect(live.rows.map((r) => r.event.text)).toContain('   1   dropped_lift, 4.2 mm from the target; found 0.6 mm / 0.8 deg off; grasp 116  (approach: tool 7.0 mm off)');
    });
});
