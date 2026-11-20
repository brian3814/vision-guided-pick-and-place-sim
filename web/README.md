# The cell's window in a browser

The interactive window of the pick-and-place cell (`pick_place/cell_app.py`) as a web page, next
to Kit's viewport streamed over WebRTC. The viewport in the page is the real one: click it, then
orbit and zoom as in Isaac Sim.

```
browser (this folder)                       Isaac Sim: cell_app.py --web
┌─────────────────────┬─────────┐
│ <video>: viewport   │ panel   │ ◄── WebRTC ──►  omni.kit.livestream.app (49100/TCP, 47998/UDP)
│ mouse/keys → Kit    │         │ ◄── JSON ────►  omni.kit.livestream.messaging
└─────────────────────┴─────────┘                    livestream.py, web_bridge.py, cell_control.py
```

## Requirements

- **Isaac Sim 6.2**, as for the rest of the project.
- **The GPU for the stream's encoder.** On a PC with integrated graphics too, Windows may list the
  integrated GPU first, and Kit's encoder (NVENC) takes that one: the stream connects, then drops at
  once, with `NV_ENC_ERR_NO_ENCODE_DEVICE` in Kit's log. In Windows Settings > System > Display >
  Graphics, add `C:\issacsim-6-2\kit\python\kit.exe` (what `python.bat` runs) and set it to **High
  performance**.
- **Node.js** 20 or later, and a Chromium browser (Chrome, Edge).
- **The client library**, `@nvidia/ov-web-rtc` 6.x, is on NVIDIA's npm registry, not on npmjs.com;
  `.npmrc` here points `@nvidia` packages at it.

## Run it

```powershell
cd web
npm install                       # once
C:\issacsim-6-2\python.bat ..\pick_place\cell_app.py --web    # in another terminal, from the repo
npm run dev                       # then open http://localhost:5173
```

`cell_app.py --web` takes the window's options (`--perception camera`, `--grippers`, ...). The page
can be opened first: it waits for Kit's stream, which starts while the cell is being built, and the
panel fills in once Kit's log says `the web page can connect over the stream`. For Kit on another machine, open
`http://localhost:5173/?server=<kit-host>` (and `&port=` for another signalling port); the streaming
ports, 49100/TCP and 47998/UDP, must be open there.

One page at a time: Kit streams to one client. A reload can take up to half a minute to reconnect,
while Kit closes the previous stream.

## Use it

- **The viewport:** click it to give it the mouse wheel and the keyboard, then use Isaac Sim's
  controls: Alt + left drag orbits, the wheel zooms. Clicking the panel takes them back; what is
  typed into the panel never reaches Kit.
- **The panel:** as Kit's (see `pick_place/README.md`, *The interactive window*): gripper, object,
  start pose (sent on Enter or when the field is left), *Run 1 cycle*, *Run 5 cycles*, *Stop after
  this cycle*, the cycle's latest step, and the results so far. While cycles run or the cell is
  being built, the controls are off; an action Kit refuses says why under the status.
- **Reloading the page** keeps the results: the page asks Kit for every event of the session so far.

**Fixed on 2026-11-20: cycles crashed Isaac Sim while a page was connected.** The stream's CUDA work
in Kit's process faulted the planner, cuMotion, whose data lives in CUDA managed memory. cuMotion
now plans in a process of its own, in every mode. The computation is the same, and so are the results.
`docs/web-stream-cumotion-crash.md` tells the whole story: the symptom, how it was narrowed down,
the cause and the fix.

**Still to do:** the plans at the start of a cycle hold up Kit for 5 to 6 s, in which Kit sends no
frame. A page loses the stream after about 7 s of that, and then needs a reload (up to half a minute).

Tested on 2026-11-20, with the cell offering all six grippers: the page, opened before Isaac Sim,
connected once the stream was up (Kit was ready 40 s after launch) and showed the viewport and Kit's
status line; the mouse wheel zoomed Kit's camera; a reload reconnected in 32 s and got the state
again.

## Develop

```sh
npm test           # vitest: the link, the results list, and the contract with Kit's bridge
npm run build      # type-check, and bundle into dist/
```

- `src/kit/`: the stream and the link to Kit. `streamer.ts` is the only file that uses NVIDIA's
  library; `link.ts` matches requests to answers; `protocol.ts` lists the messages, after
  `pick_place/web_bridge.py`; `keyboard.ts` keeps what is typed into the page out of the stream.
- `src/cell/`: `Panel.tsx` draws Kit's state and asks Kit to act; `results.ts` turns Kit's events
  into the results list, by the rules of Kit's panel.
- `src/cell/contract.json` is one session of messages recorded from the real bridge and controller
  (with the tests' fake cell); `contract.test.ts` plays it through the page's link and checks every
  state, event and answer against `protocol.ts`, field by field. The Python test that writes it
  fails when the bridge's messages change; then write it again and run the page's tests:

  ```sh
  python pick_place/tests/test_web_contract.py --write    # from the repo
  cd web && npm test
  ```
- Kit renders at its own size (1280 × 720 by default), which the page scales to fit. The library's
  `fitStreamResolution` is left out: it kept the stream from connecting.
