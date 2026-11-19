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

Tested on 2026-11-20, with the cell offering all six grippers: the page, opened before Isaac Sim,
connected once the stream was up (Kit was ready 40 s after launch) and showed the viewport and Kit's
status line; the mouse wheel zoomed Kit's camera; a reload reconnected in 32 s and got the state
again.

## Develop

```sh
npm test           # vitest
npm run build      # type-check, and bundle into dist/
```

- `src/kit/`: the stream and the link to Kit. `streamer.ts` is the only file that uses NVIDIA's
  library; `link.ts` matches requests to answers; `protocol.ts` lists the messages, after
  `pick_place/web_bridge.py`; `keyboard.ts` keeps what is typed into the page out of the stream.
- Kit renders at its own size (1280 × 720 by default), which the page scales to fit. The library's
  `fitStreamResolution` is left out: it kept the stream from connecting.
