"""remote_python's client and connection file, against a stand-in for Kit's Python server: no Isaac
Sim needed.

    python -m unittest discover -s pick_place/tests
"""

import contextlib
import io
import json
import os
import pathlib
import socket
import sys
import tempfile
import threading
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import remote_python  # noqa: E402

TOKEN = "made-at-start"
SILENT = object()  # a server that answers nothing, until the test ends


class FakeServer:
    """Listens on 127.0.0.1 like Kit's server: reads a request to its end, answers (a dict, as JSON;
    b"": closes without a word; SILENT), and keeps what it was sent."""

    def __init__(self, answer):
        self.answer = answer
        self.requests: list[dict] = []
        self.ended = threading.Event()
        self.sock = socket.create_server((remote_python.HOST, 0))
        self.port = self.sock.getsockname()[1]
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def serve(self):
        try:
            conn, _ = self.sock.accept()
        except OSError:  # closed before anyone came
            return
        with conn:
            data = b"".join(iter(lambda: conn.recv(65536), b""))  # until the client's half-close
            self.requests.append(json.loads(data))
            if self.answer is SILENT:
                self.ended.wait(10)
            elif self.answer:
                conn.sendall(json.dumps(self.answer).encode())

    def close(self):
        self.ended.set()
        self.sock.close()


class RemotePythonTest(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.folder = pathlib.Path(folder.name)
        self.saved, remote_python.FOLDER = remote_python.FOLDER, self.folder
        self.addCleanup(setattr, remote_python, "FOLDER", self.saved)

    def serve(self, answer) -> FakeServer:
        server = FakeServer(answer)
        self.addCleanup(server.close)
        remote_python.connection_file(server.port).write_text(
            json.dumps({"port": server.port, "token": TOKEN, "pid": os.getpid()}), encoding="utf-8")
        return server

    def run_main(self, *argv: str) -> tuple[int, str, str]:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = remote_python.main(list(argv))
        return code, out.getvalue(), err.getvalue()

    def test_the_code_goes_with_the_token_from_cell_apps_file(self):
        server = self.serve({"status": "ok", "output": "", "result": 2})
        answer = remote_python.send("1 + 1", server.port)
        self.assertEqual(server.requests, [{"auth_token": TOKEN, "code": "1 + 1"}])
        self.assertEqual(answer["result"], 2)

    def test_code_that_starts_like_json_is_sent_as_code(self):
        server = self.serve({"status": "ok", "output": ""})
        remote_python.send('{"code": "no"}', server.port)
        self.assertEqual(server.requests[0]["code"], '{"code": "no"}')

    def test_what_kit_prints_and_the_value_are_shown(self):
        server = self.serve({"status": "ok", "output": "stage: /World\n", "result": {"joints": 6}})
        code, out, err = self.run_main("--port", str(server.port), "-c", "...")
        self.assertEqual((code, out, err), (0, 'stage: /World\n{"joints": 6}\n', ""))

    def test_an_error_in_kit_shows_its_traceback_and_fails(self):
        server = self.serve({"status": "error", "output": "", "ename": "NameError", "evalue": "x",
                             "traceback": ["Traceback (most recent call last):\n", "NameError: x\n"]})
        code, out, err = self.run_main("--port", str(server.port), "-c", "x")
        self.assertEqual((code, out), (1, ""))
        self.assertEqual(err, "Traceback (most recent call last):\nNameError: x\n\n")

    def test_a_wrong_token_is_reported_as_kit_says_it(self):
        server = self.serve({"status": "error", "output": "", "ename": "AuthenticationError",
                             "evalue": "Missing or invalid authentication token",
                             "traceback": ["AuthenticationError: Missing or invalid authentication token"]})
        code, _, err = self.run_main("--port", str(server.port), "-c", "1")
        self.assertEqual((code, err), (1, "AuthenticationError: Missing or invalid authentication token\n"))

    def test_code_from_a_file(self):
        server = self.serve({"status": "ok", "output": ""})
        probe = self.folder / "probe.py"
        probe.write_text("import omni.usd\nprint(1)\n", encoding="utf-8")
        self.assertEqual(self.run_main("--port", str(server.port), str(probe))[0], 0)
        self.assertEqual(server.requests[0]["code"], "import omni.usd\nprint(1)\n")

    def test_no_cell_app_with_dev(self):
        code, _, err = self.run_main("--port", "1", "-c", "1")
        self.assertEqual(code, 1)
        self.assertIn("no cell_app with --dev on port 1", err)

    def test_a_file_left_by_a_cell_app_that_has_ended(self):
        server = self.serve({"status": "ok", "output": ""})
        server.close()
        code, _, err = self.run_main("--port", str(server.port), "-c", "1")
        self.assertEqual(code, 1)
        self.assertIn("has ended", err)

    def test_kit_too_busy_to_answer(self):
        server = self.serve(SILENT)
        code, _, err = self.run_main("--port", str(server.port), "--timeout", "0.3", "-c", "1")
        self.assertEqual((code, err), (1, "no answer from Kit within 0.3 s\n"))

    def test_kit_closing_without_an_answer(self):
        server = self.serve(b"")
        code, _, err = self.run_main("--port", str(server.port), "-c", "1")
        self.assertEqual((code, err), (1, "Kit closed the connection without an answer (is it closing?)\n"))

    def test_cell_app_removes_only_its_own_file(self):
        mine, theirs = remote_python.connection_file(1), remote_python.connection_file(2)
        mine.write_text(json.dumps({"port": 1, "token": "a", "pid": os.getpid()}), encoding="utf-8")
        theirs.write_text(json.dumps({"port": 2, "token": "b", "pid": os.getpid() + 1}), encoding="utf-8")
        remote_python.withdraw(mine)
        remote_python.withdraw(theirs)
        remote_python.withdraw(self.folder / "gone.json")
        self.assertEqual((mine.exists(), theirs.exists()), (False, True))


if __name__ == "__main__":
    unittest.main()
