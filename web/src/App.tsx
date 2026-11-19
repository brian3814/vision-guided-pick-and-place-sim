import { Viewport } from './kit/Viewport';
import { useKit } from './kit/useKit';

const ignore = () => {};

/** Kit's viewport, streamed, on the left; the cell's panel on the right: for now, what Kit says. */
export function App() {
    const kit = useKit(ignore);
    const state = kit.state;

    return (
        <div className="app">
            <Viewport link={kit.link} status={kit.status} info={kit.info} />
            <aside className="panel">
                <h1>Pick-and-place cell</h1>
                <p className="faint">{kit.info}</p>
                {state && (
                    <>
                        <p className="dim">{state.info}</p>
                        <p className="status">{state.status}</p>
                    </>
                )}
            </aside>
        </div>
    );
}
