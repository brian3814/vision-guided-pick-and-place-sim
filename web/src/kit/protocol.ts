/**
 * What the page and Kit say to each other over the stream's data channel: the messages of
 * pick_place/web_bridge.py, whose docstring is the reference, carrying the state and events of
 * pick_place/cell_control.py.
 *
 *     page -> Kit    cell:request    {action, request_id, ...}            one per user action
 *     Kit -> page    cell:response   {ok, error, state, request_id, ...}  whether the action was taken
 *     Kit -> page    cell:event      {event}    each controller event but state: heading, result, warning
 *     Kit -> page    cell:state      {state}    the controller's state, whenever it changed
 */

export const PROTOCOL = 1;
export const REQUEST = 'cell:request';
export const RESPONSE = 'cell:response';
export const EVENT = 'cell:event';
export const STATE = 'cell:state';

/** The actions, and what each takes: hello, gripper {name}, object {name}, pose {pose}, run {n}, stop. */
export type Action = 'hello' | 'gripper' | 'object' | 'pose' | 'run' | 'stop';

export interface Choice {
    name: string;
    /** As the list shows it: the label, how many objects or grasps, whether batch-tested. */
    text: string;
}

export interface GripperChoice extends Choice {
    /** False without a hand-eye calibration (with the camera): shown, not chosen. */
    usable: boolean;
}

/** What the controls show (CellControl.state). */
export interface CellState {
    perception: 'truth' | 'camera' | 'ros' | 'blind';
    /** Where the object's pose comes from, and why the gripper cannot change if it cannot. */
    info: string;
    gripper: string;
    object: string;
    /** The next cycle's start pose. */
    pose: number;
    /** Cycles run, or the cell is being built: the controls are off. */
    busy: boolean;
    stopping: boolean;
    fixed_gripper: boolean;
    /** The cycle's latest step, or what the window says. */
    status: string;
    grippers: GripperChoice[];
    /** The objects offered with the chosen gripper. */
    objects: Choice[];
}

/** Every event carries the time it was sent, in seconds since the epoch. */
interface Stamped {
    at: number;
}

/** A gripper x object starts in the results list. */
export interface HeadingEvent extends Stamped {
    type: 'heading';
    gripper: string;
    object: string;
    text: string;
}

/** One cycle's result (cell_control.RESULT_FIELDS), with the line the panel shows. */
export interface ResultEvent extends Stamped {
    type: 'result';
    pose: number;
    ok: boolean;
    text: string;
    outcome: string;
    place_error_m: number | null;
    tilt_deg: number | null;
    est_centre_mm: number | null;
    est_rot_deg: number | null;
    grasp_index: number | null;
    detail: string | null;
    cycle_s: number | null;
}

/** Something put right before a cycle (the wrist camera moved in the stage). */
export interface WarningEvent extends Stamped {
    type: 'warning';
    text: string;
}

export type CellEvent = HeadingEvent | ResultEvent | WarningEvent;

/** Kit's answer to a request, with the request's `request_id`. `ok` is missing when the request failed in
 * Kit's messaging itself. */
export interface Answer {
    request_id?: string;
    ok?: boolean;
    error?: string;
    state?: CellState;
    /** hello: the protocol Kit speaks, and the events so far. */
    protocol?: number;
    log?: CellEvent[];
}
