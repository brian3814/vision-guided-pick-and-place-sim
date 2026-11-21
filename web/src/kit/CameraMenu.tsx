/**
 * The camera Kit's viewport looks through, chosen as in Isaac Sim's camera menu (the page has no
 * Kit menus): Kit's Perspective, Top, Front and Right, and the stage's own cameras (the wrist camera,
 * with it). On the viewport's top left, as in Isaac Sim. It can be used at any time, even while
 * cycles run: it changes only what the stream shows.
 */
import { useEffect, useState } from 'react';
import type { ViewState } from './protocol';
import type { Kit } from './useKit';

interface Props {
    kit: Kit;
    view: ViewState;
}

export function CameraMenu({ kit, view }: Props) {
    const actual = view.camera ?? '';
    const [shown, setShown] = useState(actual);
    const [problem, setProblem] = useState('');
    useEffect(() => setShown(actual), [actual]);
    if (view.cameras.length === 0) return null;

    const choose = (path: string) => {
        setShown(path);
        setProblem('');
        const undo = (why: string) => {
            setProblem(why);
            setShown(actual);
        };
        kit.request('camera', { path })
            .then((answer) => {
                if (!answer.ok) undo(answer.error ?? 'Kit could not switch the camera');
            })
            .catch((error: Error) => undo(error.message));
    };

    return (
        <div className="camera-menu">
            <label>
                Camera
                <select value={shown} disabled={kit.status !== 'live'} onChange={(e) => choose(e.target.value)}>
                    {view.cameras.map((c) => (
                        <option key={c.path} value={c.path} title={c.path}>
                            {c.name}
                        </option>
                    ))}
                </select>
            </label>
            {problem && <p className="bad">{problem}</p>}
        </div>
    );
}
