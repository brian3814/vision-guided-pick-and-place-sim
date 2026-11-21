/**
 * The stage's unit of length, under the camera menu: from Kit, named as Kit's unit box names it
 * (the stream leaves the box out). Hovering over it says it in full. Nothing more: the stream shows
 * neither Kit's grid nor its axes, which Kit's own pictures of the viewport do.
 */

interface Props {
    /** The stage's unit (ViewState.unit): null without a stage, missing from an older Kit (protocol 2). */
    unit?: string | null;
}

const NAMES: Record<string, string> = {
    mm: 'millimetres', cm: 'centimetres', dm: 'decimetres', m: 'metres', km: 'kilometres',
    in: 'inches', ft: 'feet', mi: 'miles',
};

export function Legend({ unit }: Props) {
    if (!unit) return null;
    const name = NAMES[unit] ? `${NAMES[unit]} (${unit})` : `units of ${unit}`;
    return (
        <p className="legend" title={`Lengths in the stage are in ${name}.`}>
            unit {unit}
        </p>
    );
}
