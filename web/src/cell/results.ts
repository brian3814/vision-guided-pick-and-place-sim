/**
 * The results list, from Kit's events, by the rules of Kit's panel (cell_app.py's Panel.on_event):
 * a heading starts a gripper x object, each result is a line in green when placed (red when not),
 * and a warning is a red line. A page that says hello gets every event so far, which replace the list.
 */
import type { CellEvent } from '../kit/protocol';

export interface Row {
    key: number;
    event: CellEvent;
}

export interface Results {
    rows: Row[];
    /** The next row's key. */
    next: number;
}

export const EMPTY: Results = { rows: [], next: 0 };

export type ResultsAction =
    /** Events as Kit sends them. */
    | { type: 'add'; events: CellEvent[] }
    /** Every event so far: hello's log. */
    | { type: 'replace'; events: CellEvent[] };

export function reduce(results: Results, action: ResultsAction): Results {
    const base = action.type === 'replace' ? { rows: [], next: results.next } : results;
    if (action.events.length === 0 && action.type === 'add') return results;
    const rows = action.events.map((event, i) => ({ key: base.next + i, event }));
    return { rows: [...base.rows, ...rows], next: base.next + rows.length };
}

/** How a row is drawn: a heading, or a line that went well (`good`) or did not (`bad`). */
export function look(event: CellEvent): 'heading' | 'good' | 'bad' {
    if (event.type === 'heading') return 'heading';
    if (event.type === 'result') return event.ok ? 'good' : 'bad';
    return 'bad';
}

/** A result's details for its tooltip: when it ended and how long it took. */
export function details(event: CellEvent): string {
    const time = new Date(event.at * 1000).toLocaleTimeString();
    if (event.type !== 'result') return time;
    return event.cycle_s === null ? time : `${time}, cycle ${event.cycle_s.toFixed(1)} s`;
}
