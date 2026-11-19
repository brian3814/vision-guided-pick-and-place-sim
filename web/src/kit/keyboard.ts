/**
 * Keep what is typed into the page out of the stream.
 *
 * NVIDIA's streaming library forwards keys to Kit from a listener on `document` (bubble phase)
 * whenever its keyboard input is on, whatever element they were typed into: a start pose typed
 * into the panel would also reach Kit as shortcuts (F frames the selection).
 * Keys from editable elements stop at `document.body`, after React's handlers on #root have
 * had them and before the library's listener. Keys typed into the video still reach Kit.
 */

const EDITABLE = 'input, textarea, select, [contenteditable=""], [contenteditable="true"]';
const KEY_EVENTS = ['keydown', 'keyup', 'keypress'] as const;

export function keepTypingOutOfTheStream(boundary: HTMLElement = document.body): () => void {
    const stop = (event: Event) => {
        if (event.target instanceof Element && event.target.closest(EDITABLE)) event.stopPropagation();
    };
    for (const type of KEY_EVENTS) boundary.addEventListener(type, stop);
    return () => {
        for (const type of KEY_EVENTS) boundary.removeEventListener(type, stop);
    };
}
