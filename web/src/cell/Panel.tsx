/**
 * The cell's panel, as Kit's (cell_app.py's Panel): gripper, object, start pose, Run 1 cycle,
 * Run 5 cycles, Stop after this cycle, the cycle's latest step, and the results so far. It draws
 * Kit's state and events; its controls ask Kit to act, and Kit's state then says what happened.
 */
import { useEffect, useRef, useState } from 'react';
import type { KeyboardEvent } from 'react';
import type { Kit } from '../kit/useKit';
import type { Action, CellState } from '../kit/protocol';
import { details, look } from './results';
import type { Results } from './results';

interface Props {
    kit: Kit;
    results: Results;
}

export function Panel({ kit, results }: Props) {
    return (
        <aside className="panel">
            <h1>Pick-and-place cell</h1>
            {kit.state ? <Controls kit={kit} state={kit.state} /> : <p className="faint">{kit.info}</p>}
            <p className="dim">Results (start pose: outcome, place error; with the camera: how far off the found pose was)</p>
            <ResultsList results={results} />
        </aside>
    );
}

/**
 * What a list shows: what Kit has, except while Kit is busy, when it keeps what the user chose (a
 * gripper being built stays shown), as Kit's panel does. `stamp` changes whenever Kit says something
 * new; an answer that repeats the state changes nothing.
 */
function useShown(actual: string, busy: boolean, stamp: string): [string, (value: string) => void] {
    const [shown, setShown] = useState(actual);
    useEffect(() => {
        if (!busy) setShown(actual);
    }, [actual, busy, stamp]);
    return [shown, setShown];
}

function Controls({ kit, state }: { kit: Kit; state: CellState }) {
    const [problem, setProblem] = useState('');
    const [gripper, showGripper] = useShown(state.gripper, state.busy, state.status);
    const [object, showObject] = useShown(state.object, state.busy, state.status);
    const [pose, setPose] = useState(String(state.pose));
    useEffect(() => setPose(String(state.pose)), [state.pose]);

    const live = kit.status === 'live';
    const idle = live && !state.busy;

    /** Ask Kit; say why if it would not. `undo` puts back what the control showed. */
    const act = (action: Action, args?: Record<string, unknown>, undo?: () => void) => {
        setProblem('');
        kit.request(action, args)
            .then((answer) => {
                if (answer.ok) return;
                setProblem(answer.error ?? `Kit could not ${action}`);
                undo?.();
            })
            .catch((error: Error) => {
                setProblem(error.message);
                undo?.();
            });
    };

    const chooseGripper = (name: string) => {
        showGripper(name);
        act('gripper', { name }, () => showGripper(state.gripper));
    };
    const chooseObject = (name: string) => {
        showObject(name);
        act('object', { name }, () => showObject(state.object));
    };
    const commitPose = () => {
        const n = Number(pose);
        if (pose.trim() === '' || !Number.isInteger(n) || n < 0) setPose(String(state.pose));
        else if (n !== state.pose) act('pose', { pose: n }, () => setPose(String(state.pose)));
    };
    const poseKey = (event: KeyboardEvent<HTMLInputElement>) => {
        if (event.key === 'Enter') commitPose();
        if (event.key === 'Escape') setPose(String(state.pose));
    };

    return (
        <div className="controls">
            <p className="dim">{state.info}</p>
            <label>
                Gripper
                <select value={gripper} disabled={!idle || state.fixed_gripper} onChange={(e) => chooseGripper(e.target.value)}>
                    {state.grippers.map((g) => (
                        <option key={g.name} value={g.name} disabled={!g.usable}>
                            {g.text}
                        </option>
                    ))}
                </select>
            </label>
            <label>
                Object
                <select value={object} disabled={!idle} onChange={(e) => chooseObject(e.target.value)}>
                    {state.objects.map((o) => (
                        <option key={o.name} value={o.name}>
                            {o.text}
                        </option>
                    ))}
                </select>
            </label>
            <div className="pose">
                <label>
                    Start pose
                    <input type="number" min={0} step={1} value={pose} disabled={!idle} onChange={(e) => setPose(e.target.value)}
                           onBlur={commitPose} onKeyDown={poseKey} />
                </label>
                <span className="dim">seeded: the same number puts the object at the same place and turn</span>
            </div>
            <div className="buttons">
                <button type="button" disabled={!idle} onClick={() => act('run', { n: 1 })}>Run 1 cycle</button>
                <button type="button" disabled={!idle} onClick={() => act('run', { n: 5 })}>Run 5 cycles</button>
                <button type="button" disabled={!live || !state.busy || state.stopping} onClick={() => act('stop')}>
                    Stop after this cycle
                </button>
            </div>
            <p className="status">{state.status}</p>
            {problem && <p className="bad">{problem}</p>}
        </div>
    );
}

function ResultsList({ results }: { results: Results }) {
    const box = useRef<HTMLDivElement>(null);
    useEffect(() => {
        // Show the newest line, as a log does.
        if (box.current) box.current.scrollTop = box.current.scrollHeight;
    }, [results.next]);
    return (
        <div className="results" ref={box}>
            {results.rows.map(({ key, event }) => (
                <div key={key} className={`row ${look(event)}`} title={details(event)}>
                    {event.text}
                </div>
            ))}
        </div>
    );
}
