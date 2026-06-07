from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path

from robot_controller.process_supervisor.config import ProcessConfig
from robot_controller.process_supervisor.process_supervisor import (
    ManagedProcess,
    ProcessSupervisor,
    logger,
)


def make_process_config(name: str = "example") -> ProcessConfig:
    return ProcessConfig(
        name=name,
        command=["python", "-c", "print(1)"],
        start_order=1,
        stop_order=1,
        new_terminal=False,
        terminal_command=[],
        working_dir=".",
        env_vars={},
    )


class ProcessSupervisorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmpdir.cleanup)
        self.previous_cwd = Path.cwd()
        os.chdir(self.tmpdir.name)
        self.addCleanup(os.chdir, self.previous_cwd)

    def test_process_configs_exposes_configs_without_exposing_storage(self) -> None:
        config = make_process_config()
        supervisor = ProcessSupervisor([config])

        configs = supervisor.process_configs
        self.assertEqual(configs["example"], config)

        configs.clear()
        self.assertIn("example", supervisor.process_configs)

    def test_status_keeps_dashboard_fields_flat(self) -> None:
        supervisor = ProcessSupervisor([make_process_config()])

        row = supervisor.status()["example"]

        self.assertIn("config", row)
        self.assertEqual(row["name"], "example")
        self.assertEqual(row["alive"], False)
        self.assertEqual(row["pid"], None)
        self.assertEqual(row["managed_pid"], None)
        self.assertEqual(row["returncode"], None)
        self.assertIsInstance(row["log_file"], str)
        self.assertNotIn("health", row)

    def test_stop_always_cleans_pidfile_and_log_handle(self) -> None:
        process = ManagedProcess(
            config=make_process_config(),
            pid_dir=Path(self.tmpdir.name) / "pid",
            log_dir=Path(self.tmpdir.name) / "log",
        )
        process.pidfile.parent.mkdir(parents=True)
        process.pidfile.write_text("not-a-pid", encoding="utf-8")
        process.process = _WaitRaisesProcess()
        log_handle = _FakeLogHandle()
        process._log_handle = log_handle

        with self.assertLogs(logger.name, level="WARNING") as logs:
            with self.assertRaisesRegex(RuntimeError, "wait failed"):
                process.stop()

        self.assertFalse(process.pidfile.exists())
        self.assertTrue(log_handle.closed)
        self.assertIn("Invalid pidfile", "\n".join(logs.output))


class _WaitRaisesProcess:
    pid = 12345

    def poll(self) -> int:
        return 0

    def wait(self, timeout: float | None = None) -> int:
        raise RuntimeError("wait failed")


class _FakeLogHandle:
    def __init__(self) -> None:
        self.closed = False

    def close(self) -> None:
        self.closed = True


if __name__ == "__main__":
    unittest.main()
