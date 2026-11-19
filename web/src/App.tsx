import { useReducer } from 'react';
import { Panel } from './cell/Panel';
import { EMPTY, reduce } from './cell/results';
import { Viewport } from './kit/Viewport';
import { useKit } from './kit/useKit';

/** Kit's viewport, streamed, on the left; the cell's panel on the right. */
export function App() {
    const [results, dispatch] = useReducer(reduce, EMPTY);
    const kit = useKit((events, replace) => dispatch({ type: replace ? 'replace' : 'add', events }));

    return (
        <div className="app">
            <Viewport link={kit.link} status={kit.status} info={kit.info} />
            <Panel kit={kit} results={results} />
        </div>
    );
}
