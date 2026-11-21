import { useReducer } from 'react';
import { Panel } from './cell/Panel';
import { EMPTY, reduce } from './cell/results';
import { CameraMenu } from './kit/CameraMenu';
import { Legend } from './kit/Legend';
import { Viewport } from './kit/Viewport';
import { useKit } from './kit/useKit';

/** Kit's viewport, streamed, on the left, with its camera menu and legend; the cell's panel on the right. */
export function App() {
    const [results, dispatch] = useReducer(reduce, EMPTY);
    const kit = useKit((events, replace) => dispatch({ type: replace ? 'replace' : 'add', events }));

    return (
        <div className="app">
            <Viewport link={kit.link} status={kit.status} info={kit.info}>
                {/* An older Kit (protocol 1) sends no view. */}
                {kit.state?.view && (
                    <div className="view-corner">
                        <CameraMenu kit={kit} view={kit.state.view} />
                        <Legend unit={kit.state.view.unit} />
                    </div>
                )}
            </Viewport>
            <Panel kit={kit} results={results} />
        </div>
    );
}
