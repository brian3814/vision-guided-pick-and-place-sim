import { describe, expect, it } from 'vitest';
import type { CellEvent } from '../kit/protocol';
import { EMPTY, details, look, reduce } from './results';

const heading: CellEvent = { type: 'heading', gripper: 'robotiq_2f_140', object: 'mug', text: '2F-140 x mug', at: 10 };
const warning: CellEvent = { type: 'warning', text: 'the wrist camera had been moved', at: 11 };
const result = (pose: number, ok: boolean): CellEvent => ({
    type: 'result', pose, ok, text: `   ${pose}   ${ok ? 'placed' : 'dropped_lift'}`, outcome: ok ? 'placed' : 'dropped_lift',
    place_error_m: 0.0042, tilt_deg: 0.1, est_centre_mm: null, est_rot_deg: null, grasp_index: 116, detail: null,
    cycle_s: 15.24, at: 12 + pose,
});

describe('the results list', () => {
    it('adds events as they come, each with a key of its own', () => {
        let results = reduce(EMPTY, { type: 'add', events: [heading] });
        results = reduce(results, { type: 'add', events: [warning, result(0, true)] });
        expect(results.rows.map((r) => r.event)).toEqual([heading, warning, result(0, true)]);
        expect(results.rows.map((r) => r.key)).toEqual([0, 1, 2]);
    });

    it('is replaced by hello\'s log, with keys not used before', () => {
        const before = reduce(EMPTY, { type: 'add', events: [heading, result(0, true)] });
        const after = reduce(before, { type: 'replace', events: [heading, result(0, true), result(1, false)] });
        expect(after.rows.map((r) => r.event)).toEqual([heading, result(0, true), result(1, false)]);
        expect(after.rows.map((r) => r.key)).toEqual([2, 3, 4]);
        expect(reduce(after, { type: 'replace', events: [] }).rows).toEqual([]);
    });

    it('draws headings, results that placed or not, and warnings as Kit\'s panel does', () => {
        expect([heading, result(0, true), result(1, false), warning].map(look)).toEqual(['heading', 'good', 'bad', 'bad']);
    });

    it('gives a result\'s cycle time in its details', () => {
        expect(details(result(0, true))).toMatch(/, cycle 15\.2 s$/);
        expect(details(warning)).not.toMatch(/cycle/);
    });
});
