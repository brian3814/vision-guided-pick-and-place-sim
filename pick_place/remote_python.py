"""Kit's Python from outside, while the cell runs. `cell_app.py --dev` turns on Isaac Sim's Python
server (isaacsim.code_editor.python_server, what Isaac Sim's agent skills talk to); run as a script,
this module sends it code and prints what comes back. To look into the running cell without
stopping it, or starting Isaac Sim again for each question:

    C:/issacsim-6-2/python.bat pick_place/cell_app.py --dev       (with --web, or any other option)
    python pick_place/remote_python.py -c "omni.usd.get_context().get_stage().GetPrimAtPath('/World')"
    python pick_place/remote_python.py probe.py                   (a file; '-' or none: standard input)

What is sent runs in Kit's Python, in the loop that runs the cell, between the cell's steps: it can
do whatever the cell's own code can. Looking is harmless; changing the stage while a cycle runs
changes the cycle, and then its result says nothing about the cell. Top-level `await` works. The
names defined stay for the next request, with `omni` already there.

The server listens on 127.0.0.1 only, and runs only code that carries the token cell_app makes at
each start. Once the server listens, cell_app writes its port and the token to a file in the user's
temp folder (`connection_file`), which the client reads, and removes the file at exit; Kit also
prints the token in its log. On another port (two Isaac Sims with --dev):
--/exts/isaacsim.code_editor.python_server/port=8227 for cell_app, --port 8227 here.

The client is plain Python (standard library); `enable` and `publish` are for Kit.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import secrets
import socket
import sys
import tempfile

SERVER = "isaacsim.code_editor.python_server"
SETTINGS = f"/exts/{SERVER}"
HOST = "127.0.0.1"
PORT = 8226  # the server's default
FOLDER = pathlib.Path(tempfile.gettempdir())  # the user's own


def connection_file(port: int) -> pathlib.Path:
    """Where cell_app leaves the port and token of its server on `port`."""
    return FOLDER / f"cell_app_python_server_{port}.json"


# ------------------------------------------------------------------ in Kit


def enable() -> str:
    """Turn on Kit's Python server, on 127.0.0.1 whatever the command line says, taking only code
    with a token made now (its port: the command line's, else the server's own). Returns the token.
    The server starts listening on Kit's next updates (see `publish`)."""
    import carb
    from isaacsim.core.experimental.utils.app import enable_extension

    token = secrets.token_urlsafe(32)
    settings = carb.settings.get_settings()
    settings.set(f"{SETTINGS}/host", HOST)
    settings.set(f"{SETTINGS}/require_auth", True)
    settings.set(f"{SETTINGS}/auth_token", token)
    enable_extension(SERVER)
    return token


async def publish(token: str, next_update, updates: int = 600) -> tuple[pathlib.Path | None, str]:
    """Once the server listens, leave its port and `token` for the client. Returns the file and
    what to log; no file when the server does not listen (another one on the port: Kit says why)."""
    from isaacsim.code_editor.python_server import ServerState, get_server_status

    status = get_server_status()
    for _ in range(updates):
        if status is not None and status.state not in (ServerState.STARTING, ServerState.STOPPED):
            break
        await next_update()
        status = get_server_status()
    if status is None or status.state != ServerState.RUNNING or not status.bound_endpoints:
        why = "it is not loaded" if status is None else (status.last_error or status.state.value)
        return None, f"Kit's Python server is not listening ({why}); --dev is off for this start"
    port = status.bound_endpoints[0].port
    path = connection_file(port)
    path.write_text(json.dumps({"port": port, "token": token, "pid": os.getpid()}), encoding="utf-8")
    return path, (f"Kit's Python server listens on {HOST}:{port}: python pick_place/remote_python.py"
                  f"{'' if port == PORT else f' --port {port}'} -c \"...\" sends it code")


def withdraw(path: pathlib.Path) -> None:
    """Remove the file `publish` left, unless another start has written it since."""
    try:
        if json.loads(path.read_text(encoding="utf-8")).get("pid") == os.getpid():
            path.unlink()
    except (OSError, ValueError):
        pass


# ------------------------------------------------------------------ the client


class NotRunning(Exception):
    """No cell_app with --dev on that port."""


def send(code: str, port: int = PORT, timeout: float = 60.0) -> dict:
    """Run `code` in Kit's Python; returns the server's answer: status ('ok' or 'error'), output,
    and result (an expression's value) or ename, evalue and traceback."""
    path = connection_file(port)
    try:
        token = json.loads(path.read_text(encoding="utf-8"))["token"]
    except FileNotFoundError:
        raise NotRunning(f"no cell_app with --dev on port {port} (no {path})") from None
    # A JSON envelope rather than raw code: code that starts with '{' would be taken for one.
    request = json.dumps({"auth_token": token, "code": code}).encode()
    try:
        with socket.create_connection((HOST, port), timeout=timeout) as s:
            s.sendall(request)
            s.shutdown(socket.SHUT_WR)  # the server runs the code once the request has ended
            answer = b"".join(iter(lambda: s.recv(65536), b""))
    except ConnectionRefusedError:
        raise NotRunning(f"nothing listens on {HOST}:{port}: the cell_app that left {path} has ended") from None
    if not answer:
        raise NotRunning("Kit closed the connection without an answer (is it closing?)")
    return json.loads(answer.decode())


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("file", nargs="?", help="a Python file to run in Kit; '-' or none: standard input")
    p.add_argument("-c", dest="code", help="the code to run, instead of a file")
    p.add_argument("--port", type=int, default=PORT, help=f"the server's port (default {PORT})")
    p.add_argument("--timeout", type=float, default=60.0, help="seconds to wait for Kit's answer (default 60)")
    args = p.parse_args(argv)
    if args.code is not None:
        code = args.code
    elif args.file in (None, "-"):
        code = sys.stdin.read()
    else:
        code = pathlib.Path(args.file).read_text(encoding="utf-8")
    try:
        answer = send(code, args.port, args.timeout)
    except (NotRunning, TimeoutError) as e:
        print(e if isinstance(e, NotRunning) else f"no answer from Kit within {args.timeout:g} s", file=sys.stderr)
        return 1
    if answer.get("output"):
        print(answer["output"], end="" if answer["output"].endswith("\n") else "\n")
    if answer.get("status") != "ok":
        print("".join(answer.get("traceback") or []) or f"{answer.get('ename')}: {answer.get('evalue')}",
              file=sys.stderr)
        return 1
    if answer.get("result") is not None:
        result = answer["result"]
        print(result if isinstance(result, str) else json.dumps(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
