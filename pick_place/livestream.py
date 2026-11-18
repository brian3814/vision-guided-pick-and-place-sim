"""The web page's link to the window's controller, over Kit's livestream. Kit only.

web_bridge.py answers the page; this module puts it on the stream. `enable` turns on Kit's WebRTC
stream (omni.kit.livestream.app: signalling on 49100/TCP, media on 47998/UDP) and its messaging
(omni.kit.livestream.messaging), which carries JSON messages beside the video. `connect` hooks the
bridge to the messaging: the page's requests arrive as cell:request through `observe_and_dispatch`,
whose answer goes back as cell:response; events and state go out through the extension's
`send_message`, which splits a message over 64 KiB into parts that the page's library puts back
together.

The page's client library (6.x) sends no message id, so the extension neither matches nor
de-duplicates the page's requests: web_bridge does, by the page's own `request_id`.
"""

from __future__ import annotations

from collections.abc import Callable

import carb
from isaacsim.core.experimental.utils.app import enable_extension

from web_bridge import REQUEST, RESPONSE

STREAM = "omni.kit.livestream.app"
MESSAGING = "omni.kit.livestream.messaging"
SIGNAL_PORT = 49100  # the stream's default; Kit takes the next free port if it is taken


def enable() -> None:
    """Turn on the stream and its messaging. Kit's event traces for the stream (*-NvStreamer.etl, in
    the working directory) stay off unless the command line turns them on
    (--/exts/omni.kit.livestream.app/primaryStream/enableEventTracing=true)."""
    settings = carb.settings.get_settings()
    tracing = f"/exts/{STREAM}/primaryStream/enableEventTracing"
    if settings.get(tracing) is None:  # the extension's own default, once loaded, is on
        settings.set(tracing, False)
    enable_extension(STREAM)
    enable_extension(MESSAGING)


def connect(bridge) -> Callable[[], None]:
    """Put `bridge` on the stream: its messages go to the page, and the page's requests to it.
    Returns what takes it off again."""
    import omni.kit.livestream.messaging as messaging
    from omni.kit.livestream.messaging.scripts.extension import LivestreamMessaging

    def send(event_type: str, payload: dict) -> None:
        instance = LivestreamMessaging.instance
        if instance is not None:  # None: the extension is shutting down, nobody to send to
            instance.send_message(event_type, payload)

    bridge.send = send

    @messaging.observe_and_dispatch(REQUEST, response_name=RESPONSE)
    def on_request(_event, action: str = "", **args):
        return bridge.handle(action, args)

    def disconnect() -> None:
        bridge.send = lambda _type, _payload: None
        try:
            # observe_and_dispatch has no public way back; messaging 1.3 keeps it here.
            from omni.kit.livestream.messaging.large._extension import _unregister_observe

            _unregister_observe(REQUEST)
        except Exception as e:  # noqa: BLE001  another messaging version: the handler outlives the bridge
            carb.log_warn(f"[cell] could not stop listening to {REQUEST}: {e}")

    return disconnect
