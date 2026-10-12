"""mind/systemd/install.sh (user and --system mode) and trim-boot.sh against fake
systemctl/loginctl/id/getent/chown in a fake $HOME and a fake / ($KILO_SYSROOT)."""

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
    assert os.access(SYSTEMD / "install.sh", os.X_OK) and os.access(SYSTEMD / "trim-boot.sh", os.X_OK)

def _shim(bindir: Path, name: str, body: str) -> None:
    f = bindir / name
    f.write_text("#!/bin/sh\n" + body)
    f.chmod(0o755)


@pytest.fixture
def fake_root(tmp_path):
    """A fake / with joe's home in it, and shims that make the script believe it
    is root: `id -u` says 0, chown and loginctl only record, systemctl answers
    is-enabled from a state file and records the rest."""
    root = tmp_path / "root"
    home = root / "home/joe"
    (home / "OpenCat/.venv/bin").mkdir(parents=True)
    (home / "OpenCat/.venv/bin/kilo").write_text("#!/bin/sh\n")
    (home / "OpenCat/.venv/bin/kilo").chmod(0o755)
    bindir = tmp_path / "bin"
    bindir.mkdir()
    calls = tmp_path / "calls"
    states = tmp_path / "states"
    states.write_text("")
    log = f'echo "$(basename "$0") $*" >> "{calls}"\n'
    _shim(bindir, "id", 'case "$1" in -u) echo 0 ;; -un) echo root ;; *) echo joe ;; esac\n')
    _shim(bindir, "getent", f'case "$1 $2" in "passwd joe") echo "joe:x:1000:1000::{home}:/bin/bash" ;;\n'
                            '"group dialout") echo "dialout:x:20:joe" ;; *) exit 2 ;; esac\n')
    _shim(bindir, "chown", log)
    _shim(bindir, "loginctl", log + '[ "$1" = show-user ] && echo Linger=yes\nexit 0\n')
    _shim(bindir, "systemctl", log + f'''
if [ "$1" = is-enabled ]; then
    s=$(sed -n "s/^$2=//p" "{states}"); [ -n "$s" ] || exit 1; echo "$s"
    case $s in enabled*) exit 0 ;; *) exit 1 ;; esac
fi
case "$1" in disable) n=disabled ;; mask) n=masked ;; *) exit 0 ;; esac
sed -i "/^$2=/d" "{states}"; echo "$2=$n" >> "{states}"
''')
    env = {"PATH": f"{bindir}:/usr/bin:/bin", "KILO_SYSROOT": str(root), "SUDO_USER": "joe", "HOME": "/root"}
    return root, home, env, calls, states


def _run(script, env, *args):
    return subprocess.run(["sh", str(SYSTEMD / script), *args], env=env, capture_output=True, text=True, timeout=30)


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
def test_install_system_mode(fake_root):
    root, home, env, calls, _ = fake_root
    wants = home / ".config/systemd/user/default.target.wants"
    wants.mkdir(parents=True)
    (wants / "kilo.service").symlink_to("../kilo.service")  # the old user unit, enabled
    (root / "var/lib/systemd/linger").mkdir(parents=True)
    (root / "var/lib/systemd/linger/joe").write_text("")
    p = _run("install.sh", env, "--system")
    assert p.returncode == 0, p.stderr
    unit = (root / "etc/systemd/system/kilo.service").read_text()
    for line in ("User=joe", "Group=dialout", "RuntimeDirectory=kilo", f"EnvironmentFile=-{home}/.config/kilo/env",
                 f"ExecStart={home}/OpenCat/.venv/bin/kilo run $KILO_RUN_ARGS", "WantedBy=multi-user.target"):
        assert line in unit, line
    assert "@" not in unit.replace("User=joe", "")  # every placeholder filled
    assert "KILO_PORT=/dev/ttyACM0" in (home / ".config/kilo/env").read_text() and (home / "kilo-logs").is_dir()
    log = calls.read_text().splitlines()
    assert "systemctl --user -M joe@ disable --now kilo.service" in log
    assert "loginctl disable-linger joe" in log
    assert "systemctl daemon-reload" in log and "systemctl enable kilo.service" in log
    assert not any(x.startswith("systemctl restart") for x in log)
    assert any(x.startswith("chown joe: ") and x.endswith("/.config/kilo/env") for x in log)
    (home / ".config/kilo/env").write_text("KILO_PORT=/dev/ttyAMA0\n")
    p = _run("install.sh", env, "--system", "--start")
    assert p.returncode == 0 and "unit unchanged" in p.stdout and "kept" in p.stdout, p.stderr
    assert (home / ".config/kilo/env").read_text() == "KILO_PORT=/dev/ttyAMA0\n"
    assert "systemctl restart kilo.service" in calls.read_text()


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
def test_install_system_mode_not_root_prints_the_command(fake_root, tmp_path):
    root, home, env, calls, _ = fake_root
    _shim(tmp_path / "bin", "id", 'case "$1" in -u) echo 1000 ;; -un) echo joe ;; *) echo joe ;; esac\n')
    p = _run("install.sh", {**env, "SUDO_USER": ""}, "--system", "--start")
    assert p.returncode == 1
    assert f"sudo {SYSTEMD / 'install.sh'} --system --user joe --start" in p.stderr
    assert not (root / "etc/systemd/system/kilo.service").exists()


def test_system_unit_shape():
    text = (SYSTEMD / "kilo-system.service").read_text()
    for line in ("User=@USER@", "Group=@GROUP@", "RuntimeDirectory=kilo", "EnvironmentFile=-@HOME@/.config/kilo/env",
                 "ExecStart=@KILO@ run $KILO_RUN_ARGS", "KillSignal=SIGTERM", "TimeoutStopSec=15",
                 "Restart=on-failure", "WantedBy=multi-user.target", "After=basic.target"):
        assert line in text, line
    # nothing on the path to `kilo run` waits for the network (Kilo must run with no Wi-Fi)
    directives = [x for x in text.splitlines() if x and not x.startswith("#")]
    assert not any("network" in x for x in directives if x.split("=")[0] in ("After", "Wants", "Requires",
                                                                           "BindsTo", "Requisite"))
    assert "DefaultDependencies" not in text


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
def test_trim_boot(fake_root):
    root, home, env, calls, states = fake_root
    (root / "etc/cloud").mkdir(parents=True)
    (root / "etc/netplan").mkdir(parents=True)
    (root / "etc/netplan/50-cloud-init.yaml").write_text("network: {wifis: {}}\n")
    (root / "boot/firmware").mkdir(parents=True)
    cmdline = root / "boot/firmware/cmdline.txt"
    original = "console=tty1 root=PARTUUID=x-02 rootfstype=ext4 fsck.repair=yes rootwait quiet splash " \
               "plymouth.ignore-serial-consoles cfg80211.ieee80211_regdom=GB\n"
    cmdline.write_text(original)
    states.write_text("cloud-init-local.service=enabled\ncloud-init-main.service=enabled\n"
                      "cloud-init-network.service=enabled\ncloud-final.service=enabled\n"
                      "NetworkManager-wait-online.service=enabled\nNetworkManager.service=enabled\n"
                      "tailscaled.service=enabled\nrpi-resize-swap-file.service=enabled\n")
    p = _run("trim-boot.sh", env, "--dry-run")
    assert p.returncode == 0, p.stderr
    assert "would:" in p.stdout and "would: NetworkManager-wait-online.service: masked" in p.stdout
    assert cmdline.read_text() == original
    assert all(x.split()[1] == "is-enabled" for x in calls.read_text().splitlines())  # looked only
    assert not (root / "etc/cloud/cloud-init.disabled").exists()
    p = _run("trim-boot.sh", env)
    assert p.returncode == 0, p.stderr
    assert (root / "etc/cloud/cloud-init.disabled").exists()
    st = dict(x.split("=") for x in states.read_text().split())
    for u in ("cloud-init-local", "cloud-init-main", "cloud-init-network", "cloud-final"):
        assert st[u + ".service"] == "disabled"
    assert st["NetworkManager-wait-online.service"] == "masked"
    # never touched: the network itself, tailscale, the swap file resizer
    for u in ("NetworkManager", "tailscaled", "rpi-resize-swap-file"):
        assert st[u + ".service"] == "enabled"
    assert cmdline.read_text() == original.replace(" splash", "")
    assert " quiet " in cmdline.read_text()
    assert (root / "boot/firmware/cmdline.txt.kilo-backup").read_text() == original
    p = _run("trim-boot.sh", env)  # twice: nothing left to do
    assert p.returncode == 0 and "nothing to change" in p.stdout, p.stdout + p.stderr
    assert cmdline.read_text() == original.replace(" splash", "")
    assert (root / "boot/firmware/cmdline.txt.kilo-backup").read_text() == original


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
def test_trim_boot_keeps_cloud_init_without_wifi_on_disk(fake_root):
    root, home, env, calls, states = fake_root
    (root / "etc/cloud").mkdir(parents=True)
    states.write_text("cloud-init-main.service=enabled\n")
    p = _run("trim-boot.sh", env)
    assert p.returncode == 0 and "LEFT ON" in p.stderr
    assert not (root / "etc/cloud/cloud-init.disabled").exists()
    assert "cloud-init-main.service=enabled" in states.read_text()
