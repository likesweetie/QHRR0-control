from qhrr0.app.robot_controller.process_supervisor import ProcessSupervisor


def _process_configs() -> list[dict]:
    return [
        {
            "name": "first",
            "command": ["python3", "-c", "print('first')"],
            "start_order": 10,
            "stop_order": 20,
            "new_terminal": False,
            "terminal_command": [],
            "working_dir": ".",
            "env_vars": {},
        },
        {
            "name": "second",
            "command": ["python3", "-c", "print('second')"],
            "start_order": 20,
            "stop_order": 10,
            "new_terminal": False,
            "terminal_command": [],
            "working_dir": ".",
            "env_vars": {"QHRR_TEST": "1"},
        },
    ]


def test_process_supervisor_accepts_dict_configs(monkeypatch, tmp_path) -> None:
    monkeypatch.chdir(tmp_path)

    supervisor = ProcessSupervisor(_process_configs())

    assert sorted(supervisor.process_configs) == ["first", "second"]
    assert supervisor.process_configs["first"]["command"][0] == "python3"
    assert (tmp_path / "log").is_dir()


def test_process_supervisor_status_serializes_dict_config(monkeypatch, tmp_path) -> None:
    monkeypatch.chdir(tmp_path)

    supervisor = ProcessSupervisor(_process_configs())
    status = supervisor.status()

    assert status["first"]["config"]["name"] == "first"
    assert status["first"]["alive"] is False
