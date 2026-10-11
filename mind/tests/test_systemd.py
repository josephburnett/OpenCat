"""mind/systemd/install.sh against fake systemctl/loginctl in a fake $HOME."""

import os
import shutil
import subprocess
from pathlib import Path

import pytest

SYSTEMD = Path(__file__).resolve().parents[1] / "systemd"


@pytest.fixture
def fake_home(tmp_path):
    home = tmp_path / "home"
    (home / "OpenCat/.venv/bin").mkdir(parents=True)
    kilo = home / "OpenCat/.venv/bin/kilo"
    kilo.write_text("#!/bin/sh\n")
    kilo.chmod(0o755)
    bindir = tmp_path / "bin"
    bindir.mkdir()
    calls = tmp_path / "calls"
    for tool in ("systemctl", "loginctl"):
        f = bindir / tool
        f.write_text(f'#!/bin/sh\necho "{tool} $*" >> "{calls}"\n'
                     + ('[ "$1" = show-user ] && echo Linger=no\n' if tool == "loginctl" else "") + "exit 0\n")
        f.chmod(0o755)
    env = {"HOME": str(home), "PATH": f"{bindir}:/usr/bin:/bin"}
    return home, env, calls


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
def test_install_is_idempotent_and_does_not_start(fake_home):
    home, env, calls = fake_home
    run = lambda *a: subprocess.run(["sh", str(SYSTEMD / "install.sh"), *a], env=env, capture_output=True,
                                    text=True, timeout=30)
    p = run()
    assert p.returncode == 0, p.stderr
    unit = home / ".config/systemd/user/kilo.service"
    envf = home / ".config/kilo/env"
    assert unit.read_text() == (SYSTEMD / "kilo.service").read_text()
    assert "KILO_PORT=/dev/ttyACM0" in envf.read_text() and (home / "kilo-logs").is_dir()
    log = calls.read_text().splitlines()
    assert "systemctl --user daemon-reload" in log and "systemctl --user enable kilo.service" in log
    assert any(x.startswith("loginctl enable-linger") for x in log)
    assert not any("start" in x for x in log if x.startswith("systemctl"))
    envf.write_text("KILO_PORT=/dev/ttyAMA0\n")
    p = run()
    assert p.returncode == 0 and "unit unchanged" in p.stdout and "kept" in p.stdout
    assert envf.read_text() == "KILO_PORT=/dev/ttyAMA0\n"  # never overwritten
    p = run("--start", "--kilo", "/opt/kilo/bin/kilo")
    assert p.returncode == 0
    assert "ExecStart=/opt/kilo/bin/kilo run $KILO_RUN_ARGS" in unit.read_text()
    assert "systemctl --user restart kilo.service" in calls.read_text()
    assert run("--bogus").returncode == 2


def test_unit_file_shape():
    text = (SYSTEMD / "kilo.service").read_text()
    for line in ("EnvironmentFile=-%h/.config/kilo/env", "ExecStart=%h/OpenCat/.venv/bin/kilo run $KILO_RUN_ARGS",
                 "KillSignal=SIGTERM", "WantedBy=default.target", "Restart=on-failure"):
        assert line in text
    assert os.access(SYSTEMD / "install.sh", os.X_OK)
