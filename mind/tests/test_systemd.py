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
    for script in ("install.sh", "trim-boot.sh", "safe-mode.sh", "data-partition.sh"):
        assert os.access(SYSTEMD / script, os.X_OK), script

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
                 f"ExecStart={home}/OpenCat/.venv/bin/kilo run $KILO_RUN_ARGS", "WantedBy=multi-user.target",
                 f"RequiresMountsFor={home}"):
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
                 "Restart=on-failure", "WantedBy=multi-user.target", "After=basic.target",
                 "RequiresMountsFor=@HOME@"):
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
    # the journal goes to disk (Raspberry Pi OS ships Storage=volatile)
    jconf = root / "etc/systemd/journald.conf.d/50-kilo.conf"
    assert jconf.read_text() == "[Journal]\nStorage=persistent\n"
    p = _run("trim-boot.sh", env)  # twice: nothing left to do
    assert p.returncode == 0 and "nothing to change" in p.stdout, p.stdout + p.stderr
    assert "journal: already persistent" in p.stdout
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


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
def test_trim_boot_ignores_an_empty_wifi_profile(fake_root):
    """2026-10-12: an unclean shutdown left a 0-byte netplan profile; it must not count as saved Wi-Fi."""
    root, home, env, calls, states = fake_root
    (root / "etc/cloud").mkdir(parents=True)
    (root / "etc/netplan").mkdir(parents=True)
    (root / "etc/netplan/90-NM-75a1216a.yaml").write_text("")
    (root / "etc/NetworkManager/system-connections").mkdir(parents=True)
    (root / "etc/NetworkManager/system-connections/Wired.nmconnection").write_text("[connection]\ntype=ethernet\n")
    states.write_text("cloud-init-main.service=enabled\n")
    p = _run("trim-boot.sh", env)
    assert p.returncode == 0 and "LEFT ON" in p.stderr, p.stdout + p.stderr
    assert not (root / "etc/cloud/cloud-init.disabled").exists()
    # a real keyfile Wi-Fi profile does count
    (root / "etc/NetworkManager/system-connections/strathmore.nmconnection").write_text("[connection]\n[wifi]\nssid=x\n")
    p = _run("trim-boot.sh", env, "--dry-run")
    assert p.returncode == 0 and "would: cloud-init: disabled" in p.stdout, p.stdout + p.stderr


@pytest.mark.skipif(shutil.which("sh") is None or os.geteuid() == 0, reason="needs sh and a non-root user")
def test_trim_boot_dry_run_says_when_it_cannot_read_the_profile(fake_root):
    root, home, env, calls, states = fake_root
    (root / "etc/cloud").mkdir(parents=True)
    d = root / "etc/NetworkManager/system-connections"
    d.mkdir(parents=True)
    f = d / "strathmore.nmconnection"
    f.write_text("[wifi]\nssid=x\n")
    f.chmod(0)
    try:
        p = _run("trim-boot.sh", env, "--dry-run")
    finally:
        f.chmod(0o600)
    assert p.returncode == 0 and "cannot tell" in p.stdout, p.stdout + p.stderr
    assert "cloud-init" not in " ".join(x for x in p.stdout.splitlines() if x.startswith("would:"))


# ---- safe-mode.sh -------------------------------------------------------------------------

DATA_FSTAB = """\
PARTUUID=abcd-01  /boot/firmware  vfat  defaults  0  2
PARTUUID=abcd-02  /  ext4  defaults,noatime  0  1
LABEL=kilo-data  /data  ext4  defaults,noatime,nofail  0  2
/data/home/joe  /home/joe  none  bind,nofail,x-systemd.requires=/data  0  0
/data/var/lib/tailscale  /var/lib/tailscale  none  bind,nofail,x-systemd.requires=/data  0  0
/data/var/log/journal  /var/log/journal  none  bind,nofail,x-systemd.requires=/data  0  0
/data/var/lib/systemd/timesync  /var/lib/systemd/timesync  none  bind,nofail,x-systemd.requires=/data  0  0
"""
BINDS = ("/home/joe", "/var/lib/tailscale", "/var/log/journal", "/var/lib/systemd/timesync")
CMDLINE = "console=tty1 root=PARTUUID=abcd-02 rootfstype=ext4 fsck.repair=yes rootwait quiet\n"


def _proc_mounts(overlay=False, boot_ro=False, data=True, binds=BINDS):
    lines = ["overlayroot / overlay rw,relatime,lowerdir=/media/root-ro,upperdir=/media/root-rw/overlay 0 0"
             if overlay else "/dev/mmcblk0p2 / ext4 rw,noatime 0 0",
             f"/dev/mmcblk0p1 /boot/firmware vfat {'ro' if boot_ro else 'rw'},relatime,fmask=0022 0 0"]
    if data:
        lines.append("/dev/mmcblk0p3 /data ext4 rw,noatime 0 0")
        lines += [f"/dev/mmcblk0p3 {b} ext4 rw,noatime 0 0" for b in binds]
    return "\n".join(lines) + "\n"


@pytest.fixture
def pi(fake_root):
    """The fake root as the Pi after data-partition.sh: /data and the binds mounted and in fstab.
    raspi-config's nonint functions are shimmed to make raspi-config's own edits."""
    root, home, env, calls, _ = fake_root
    bindir = Path(env["PATH"].split(":")[0])
    (root / "proc").mkdir()
    (root / "proc/mounts").write_text(_proc_mounts())
    (root / "boot/firmware").mkdir(parents=True)
    (root / "boot/firmware/cmdline.txt").write_text(CMDLINE)
    (root / "boot/firmware/initramfs7").write_text("")
    (root / "etc").mkdir(exist_ok=True)
    (root / "etc/fstab").write_text(DATA_FSTAB)
    log = f'echo "$(basename "$0") $*" >> "{calls}"\n'
    _shim(bindir, "raspi-config", log + f'''
[ "$1" = nonint ] || exit 1
case "$2" in
  enable_bootro) sed -i "{root}/etc/fstab" -e "s#\\(.*/boot/firmware.*\\)defaults\\(.*\\)#\\1defaults,ro\\2#" ;;
  disable_bootro) sed -i "{root}/etc/fstab" -e "s#\\(.*/boot/firmware.*\\)defaults,ro\\(.*\\)#\\1defaults\\2#" ;;
  enable_overlayfs) grep -q overlayroot=tmpfs "{root}/boot/firmware/cmdline.txt" || \\
      sed -i "{root}/boot/firmware/cmdline.txt" -e "s/^/overlayroot=tmpfs /" ;;
  *) exit 1 ;;
esac
''')
    _shim(bindir, "mount", log)
    # overlayroot-chroot CMD... runs CMD in the real root; here: on the fake lower root's fstab
    _shim(bindir, "overlayroot-chroot", log + f'''a=""
for x in "$@"; do [ "$x" = /etc/fstab ] && x="{root}/media/root-ro/etc/fstab"; a="$a '$x'"; done
eval "$a"
''')
    return root, env, calls


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
def test_safe_mode_status_on_a_writable_pi(pi):
    root, env, calls = pi
    p = _run("safe-mode.sh", env, "status")
    assert p.returncode == 0, p.stderr
    out = p.stdout
    assert "overlay root now:        off" in out and "overlay root next boot:  off" in out
    assert "/boot/firmware now:      writable" in out and "/boot/firmware next boot: writable" in out
    assert "/data:                   mounted" in out
    for b in BINDS:
        assert any(x.startswith(b + ":") and "mounted, in fstab" in x for x in out.splitlines()), b
    assert "warning" not in out
    assert not calls.exists()  # status only reads


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
@pytest.mark.parametrize("mounts, fstab, why", [
    (_proc_mounts(data=False), DATA_FSTAB, "/data is not mounted"),
    (_proc_mounts(binds=BINDS[1:]), DATA_FSTAB, "/home/joe is not bind-mounted"),
    (_proc_mounts(), DATA_FSTAB.replace("/data/var/lib/tailscale  /var/lib/tailscale", "#"),
     "/var/lib/tailscale has no line in /etc/fstab"),
])
def test_safe_mode_on_refused_without_data(pi, mounts, fstab, why):
    root, env, calls = pi
    (root / "proc/mounts").write_text(mounts)
    (root / "etc/fstab").write_text(fstab)
    for args in (("on",), ("on", "--dry-run")):
        p = _run("safe-mode.sh", env, *args)
        assert p.returncode == 1 and "REFUSED" in p.stderr and why in p.stderr, p.stdout + p.stderr
    assert not calls.exists()
    assert (root / "boot/firmware/cmdline.txt").read_text() == CMDLINE


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
def test_safe_mode_on_then_off(pi):
    root, env, calls = pi
    cmdline = root / "boot/firmware/cmdline.txt"
    p = _run("safe-mode.sh", env, "on", "--dry-run")
    assert p.returncode == 0, p.stderr
    assert "would: /boot/firmware: read-only" in p.stdout and "raspi-config nonint enable_overlayfs" in p.stdout
    assert "overlayroot=tmpfs:recurse=0 console=tty1" in p.stdout
    assert not calls.exists() and cmdline.read_text() == CMDLINE
    assert (root / "etc/fstab").read_text() == DATA_FSTAB

    p = _run("safe-mode.sh", env, "on")
    assert p.returncode == 0, p.stderr
    assert "reboot to apply" in p.stdout
    assert calls.read_text().splitlines() == ["raspi-config nonint enable_bootro",
                                              "raspi-config nonint enable_overlayfs"]
    # raspi-config's plain overlayroot=tmpfs (recurse=1 would overlay /data too) became recurse=0
    assert cmdline.read_text() == "overlayroot=tmpfs:recurse=0 " + CMDLINE
    assert (root / "boot/firmware/cmdline.txt.kilo-safe-mode-backup").read_text() == "overlayroot=tmpfs " + CMDLINE
    assert "/boot/firmware  vfat  defaults,ro  0  2" in (root / "etc/fstab").read_text()
    out = _run("safe-mode.sh", env, "status").stdout
    assert "overlay root now:        off" in out and "overlay root next boot:  ON  (overlayroot=tmpfs:recurse=0" in out
    assert "/boot/firmware next boot: read-only" in out

    p = _run("safe-mode.sh", env, "on")  # again: nothing to do
    assert p.returncode == 0 and "nothing to change" in p.stdout, p.stdout + p.stderr
    assert len(calls.read_text().splitlines()) == 2

    p = _run("safe-mode.sh", env, "off")  # before the reboot: the plain reverse
    assert p.returncode == 0, p.stderr
    assert cmdline.read_text() == CMDLINE
    assert (root / "etc/fstab").read_text() == DATA_FSTAB
    assert calls.read_text().splitlines()[-1] == "raspi-config nonint disable_bootro"
    p = _run("safe-mode.sh", env, "off")
    assert p.returncode == 0 and "nothing to change" in p.stdout


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
def test_safe_mode_off_while_the_overlay_is_active(pi):
    """In safe mode /etc/fstab is overlayroot's copy in RAM and raspi-config refuses bootro changes:
    the boot partition is remounted rw for the kernel line, and the real fstab is edited via
    overlayroot-chroot."""
    root, env, calls = pi
    ro_fstab = DATA_FSTAB.replace("vfat  defaults ", "vfat  defaults,ro ")
    (root / "media/root-ro/etc").mkdir(parents=True)
    (root / "media/root-ro/etc/fstab").write_text(ro_fstab)
    (root / "etc/fstab").write_text("# overlayroot's generated copy\n" + DATA_FSTAB)
    (root / "proc/mounts").write_text(_proc_mounts(overlay=True, boot_ro=True))
    cmdline = root / "boot/firmware/cmdline.txt"
    cmdline.write_text("overlayroot=tmpfs:recurse=0 " + CMDLINE)
    out = _run("safe-mode.sh", env, "status").stdout
    assert "overlay root now:        ON" in out and "/boot/firmware now:      read-only" in out
    assert "/boot/firmware next boot: read-only" in out  # read from the real root's fstab
    p = _run("safe-mode.sh", env, "on")
    assert p.returncode == 0 and "nothing to change" in p.stdout, p.stdout + p.stderr
    p = _run("safe-mode.sh", env, "off")
    assert p.returncode == 0, p.stderr
    log = calls.read_text().splitlines()
    assert log[:2] == ["mount -o remount,rw /boot/firmware", "mount -o remount,ro /boot/firmware"]
    assert log[2].startswith("overlayroot-chroot sed -i -e ")
    assert not any(x.startswith("raspi-config") for x in log)
    assert cmdline.read_text() == CMDLINE
    assert (root / "media/root-ro/etc/fstab").read_text() == DATA_FSTAB
    out = _run("safe-mode.sh", env, "status").stdout
    assert "overlay root next boot:  off" in out and "/boot/firmware next boot: writable" in out


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
def test_safe_mode_flags_a_plain_overlayroot_and_fixes_it(pi):
    root, env, calls = pi
    cmdline = root / "boot/firmware/cmdline.txt"
    cmdline.write_text("overlayroot=tmpfs " + CMDLINE)  # what raspi-config alone writes
    assert "lacks recurse=0" in _run("safe-mode.sh", env, "status").stdout
    p = _run("safe-mode.sh", env, "on")
    assert p.returncode == 0, p.stderr
    assert cmdline.read_text() == "overlayroot=tmpfs:recurse=0 " + CMDLINE
    assert "enable_overlayfs" not in calls.read_text()


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
def test_safe_mode_needs_root_and_a_command(pi, tmp_path):
    root, env, calls = pi
    assert _run("safe-mode.sh", env).returncode == 2
    assert _run("safe-mode.sh", env, "sideways").returncode == 2
    _shim(tmp_path / "bin", "id", 'case "$1" in -u) echo 1000 ;; *) echo joe ;; esac\n')
    p = _run("safe-mode.sh", env, "on")
    assert p.returncode == 1 and "sudo" in p.stderr
    assert _run("safe-mode.sh", env, "on", "--dry-run").returncode == 0
    assert _run("safe-mode.sh", env, "status").returncode == 0


# ---- data-partition.sh --------------------------------------------------------------------
# Only the plan and the refusals can be checked for real. The full run below goes through
# shims (lsblk, blkid, dumpe2fs, e2fsck, resize2fs, sfdisk, mkfs.ext4, mount) that model a card
# as directories and size files: it checks the order of steps, the copies, fstab and
# idempotence, not what sfdisk/resize2fs/mkfs do to a real card.

GIB = 1 << 30


@pytest.fixture
def card(tmp_path):
    f = tmp_path / "card"
    (f / "sdz1").mkdir(parents=True)
    (f / "sdz1/cmdline.txt").write_text("console=tty1 root=PARTUUID=abcd-02 rootwait quiet resize\n")
    p2 = f / "sdz2"
    (p2 / "etc").mkdir(parents=True)
    (p2 / "etc/fstab").write_text("PARTUUID=abcd-01  /boot/firmware  vfat  defaults  0  2\n"
                                  "PARTUUID=abcd-02  /  ext4  defaults,noatime  0  1\n")
    (p2 / "home/joe/OpenCat").mkdir(parents=True)
    (p2 / "home/joe/OpenCat/README.md").write_text("kilo\n")
    (p2 / "home/joe/.ssh").mkdir(mode=0o700)
    (p2 / "home/joe/.ssh/authorized_keys").write_text("ssh-ed25519 AAAA\n")
    (p2 / "home/joe/.config/kilo").mkdir(parents=True)
    (p2 / "home/joe/.config/kilo/env").write_text("KILO_RUN_ARGS=--remote\n")
    (p2 / "var/lib/tailscale").mkdir(parents=True)
    (p2 / "var/lib/tailscale/tailscaled.state").write_text("{}\n")
    (p2 / "var/lib/systemd/timesync").mkdir(parents=True)
    (p2 / "var/lib/systemd/timesync/clock").write_text("")
    # no var/log/journal: created empty
    st = f / "state"
    st.mkdir()
    for k, v in {"size.sdz": 59 * GIB, "size.sdz1": 512 << 20, "size.sdz2": 58 * GIB,
                 "fstype.sdz1": "vfat", "fstype.sdz2": "ext4",
                 "blocks.sdz2": 58 * GIB // 4096, "free.sdz2": (58 * GIB - 6 * GIB) // 4096}.items():
        (st / k).write_text(f"{v}\n")
    bindir = tmp_path / "cbin"
    bindir.mkdir()
    calls = tmp_path / "card-calls"
    log = f'echo "$(basename "$0") $*" >> "{calls}"\n'
    last = 'eval last=\\${$#}; n=$(basename "$last")\n'
    _shim(bindir, "id", 'case "$1" in -u) echo 0 ;; *) echo root ;; esac\n')
    _shim(bindir, "findmnt", 'echo /dev/nvme0n1p2\n')
    _shim(bindir, "lsblk", last + f'''
case "$*" in
  *-nrpso*) echo "/dev/nvme0n1p2 part"; echo "${{ROOTDISK:-/dev/nvme0n1}} disk" ;;
  *"-dno TYPE"*) if [ "$n" = sdz ]; then echo disk; elif [ -e "{st}/size.$n" ]; then echo part; fi ;;
  *"-nrpo NAME,MOUNTPOINT"*) echo /dev/sdz
      for p in "{st}"/size.sdz?; do q=${{p##*.}}; echo "/dev/$q $(cat "{st}/mp.$q" 2>/dev/null)"; done ;;
  *"-nrpo NAME,TYPE"*) echo "/dev/sdz disk"; for p in "{st}"/size.sdz?; do echo "/dev/${{p##*.}} part"; done ;;
  *LOG-SEC*) echo 512 ;;
  *"-bdno SIZE"*) cat "{st}/size.$n" ;;
  *) echo "NAME SIZE TYPE (fake lsblk of $n)" ;;
esac
''')
    _shim(bindir, "blkid", last + f'case "$*" in *TYPE*) cat "{st}/fstype.$n" 2>/dev/null ;; '
                                    f'*LABEL*) cat "{st}/label.$n" 2>/dev/null ;; esac; exit 0\n')
    _shim(bindir, "dumpe2fs", last + f'echo "Block count:              $(cat "{st}/blocks.$n")"\n'
                                       'echo "Block size:               4096"\n'
                                       f'echo "Free blocks:              $(cat "{st}/free.$n")"\n')
    _shim(bindir, "e2fsck", log)
    _shim(bindir, "resize2fs", log + f'n=$(basename "$1"); g=${{2%G}}; echo $((g * 262144)) > "{st}/blocks.$n"\n'
                                     f'echo $(( g * 262144 - 6 * 262144 )) > "{st}/free.$n"\n')
    _shim(bindir, "sfdisk", log + f'''read line; echo "  stdin: $line" >> "{calls}"
case "$1" in
  -N) echo $(( ${{line#, }} * 512 )) > "{st}/size.sdz2" ;;
  --append) echo $(( 59 * {GIB} - $(cat "{st}/size.sdz2") - (513 << 20) )) > "{st}/size.sdz3"; mkdir -p "{f}/sdz3" ;;
esac
''')
    _shim(bindir, "mkfs.ext4", log + last + f'echo ext4 > "{st}/fstype.$n"; echo "$3" > "{st}/label.$n"\n')
    _shim(bindir, "udevadm", "exit 0\n")
    # mount -o OPTS DEV DIR: DIR becomes a link to the partition's directory
    _shim(bindir, "mount", log + f'rmdir "$4" && ln -s "{f}/$(basename "$3")" "$4"\n')
    _shim(bindir, "umount", 'rm "$1" && mkdir "$1"\n')
    scratch = tmp_path / "scratch"
    scratch.mkdir()
    env = {"PATH": f"{bindir}:/usr/bin:/bin", "TMPDIR": str(scratch), "HOME": "/root"}
    return f, st, env, calls, scratch


def _card_tree(f):
    return {str(p.relative_to(f)): (p.read_text() if p.is_file() else None)
            for p in sorted(f.rglob("*")) if "state" not in p.parts}


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
@pytest.mark.parametrize("args", [("/dev/sdz", "--dry-run"), ("/dev/sdz",), ("/dev/sdz", "--dry-run", "--yes")])
def test_data_partition_plan_changes_nothing(card, args):
    f, st, env, calls, scratch = card
    before = _card_tree(f)
    p = _run("data-partition.sh", env, *args)
    assert p.returncode == 0, p.stdout + p.stderr
    out = p.stdout
    assert "fake lsblk of sdz" in out  # the card is shown
    assert "resize2fs /dev/sdz2 16G" in out and "p2 size 33554432 sectors" in out
    assert "mkfs.ext4 -L kilo-data /dev/sdz3" in out and "drop the word 'resize'" in out
    assert "/home/joe" in out and "/var/log/journal  (not on the card: created empty)" in out
    assert "--yes" in out.splitlines()[-1]
    assert [x.split()[0] for x in calls.read_text().splitlines()] == ["mount", "mount"]  # look only
    assert all(" -o ro " in x for x in calls.read_text().splitlines())
    assert _card_tree(f) == before
    assert list(scratch.iterdir()) == []  # mount points cleaned up


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
@pytest.mark.parametrize("setup, args, why", [
    (lambda st, env: (st / "mp.sdz1").write_text("/media/joe/bootfs"), (), "is mounted"),
    (lambda st, env: env.update(ROOTDISK="/dev/sdz"), (), "this host's own root"),
    (lambda st, env: None, ("/dev/sdz1",), "not a whole disk"),
    (lambda st, env: (st / "fstype.sdz2").write_text("btrfs"), (), "/dev/sdz2 is not ext4"),
    (lambda st, env: (st / "fstype.sdz1").write_text("ntfs"), (), "/dev/sdz1 is not vfat"),
    (lambda st, env: ((st / "size.sdz3").write_text("100"), (st / "fstype.sdz3").write_text("ntfs")), (),
     "unknown layout"),
    (lambda st, env: (st / "free.sdz2").write_text("100"), (), "Use a bigger --root-size"),
])
def test_data_partition_refusals(card, setup, args, why):
    f, st, env, calls, scratch = card
    setup(st, env)
    before = _card_tree(f)
    p = _run("data-partition.sh", env, *(args or ("/dev/sdz",)), "--yes")
    assert p.returncode == 1 and "REFUSED" in p.stderr and why in p.stderr, p.stdout + p.stderr
    log = calls.read_text() if calls.exists() else ""
    assert not any(t in log for t in ("e2fsck", "resize2fs", "sfdisk", "mkfs"))
    assert " -o rw " not in log
    assert _card_tree(f) == before
    assert list(scratch.iterdir()) == []


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
def test_data_partition_usage_and_root(card, tmp_path):
    f, st, env, calls, scratch = card
    assert _run("data-partition.sh", env).returncode == 2
    assert _run("data-partition.sh", env, "/dev/sdz", "--root-size", "big").returncode == 2
    _shim(tmp_path / "cbin", "id", 'case "$1" in -u) echo 1000 ;; *) echo joe ;; esac\n')
    p = _run("data-partition.sh", env, "/dev/sdz", "--dry-run")
    assert p.returncode == 1 and "sudo" in p.stderr
    assert not calls.exists()


@pytest.mark.skipif(shutil.which("sh") is None, reason="needs sh")
def test_data_partition_full_run_then_again(card):
    f, st, env, calls, scratch = card
    p = _run("data-partition.sh", env, "/dev/sdz", "--yes")
    assert p.returncode == 0, p.stdout + p.stderr
    steps = [x for x in calls.read_text().splitlines() if not x.startswith(("mount", "  stdin"))]
    assert steps == ["e2fsck -f -p /dev/sdz2", "resize2fs /dev/sdz2 16G", "sfdisk -N 2 /dev/sdz",
                     "sfdisk --append /dev/sdz", "mkfs.ext4 -q -L kilo-data /dev/sdz3"]
    assert "  stdin: , 33554432" in calls.read_text() and "  stdin: ,,L" in calls.read_text()
    assert (st / "size.sdz2").read_text().strip() == str(16 * GIB)
    d = f / "sdz3"
    assert (d / "home/joe/.config/kilo/env").read_text() == "KILO_RUN_ARGS=--remote\n"
    assert (d / "home/joe/.ssh/authorized_keys").exists() and (d / "home/joe/.ssh").stat().st_mode & 0o777 == 0o700
    assert (d / "var/lib/tailscale/tailscaled.state").exists() and (d / "var/lib/systemd/timesync/clock").exists()
    assert (d / "var/log/journal").is_dir() and (f / "sdz2/var/log/journal").is_dir()
    assert (f / "sdz2/data").is_dir()
    assert (f / "sdz2/home/joe/.config/kilo/env").exists()  # the original stays (the fallback)
    fstab = (f / "sdz2/etc/fstab").read_text()
    assert fstab.startswith("PARTUUID=abcd-01  /boot/firmware  vfat  defaults  0  2\n")
    assert "LABEL=kilo-data  /data  ext4  defaults,noatime,nofail," in fstab
    for b in BINDS:
        assert f"/data{b}  {b}  none  bind,nofail,x-systemd.requires=/data," in fstab, b
    assert (f / "sdz2/etc/fstab.kilo-data-backup").exists()
    assert (f / "sdz1/cmdline.txt").read_text() == "console=tty1 root=PARTUUID=abcd-02 rootwait quiet\n"
    assert list(scratch.iterdir()) == []

    (d / "home/joe/.config/kilo/env").write_text("changed on the Pi\n")
    n = len(calls.read_text().splitlines())
    p = _run("data-partition.sh", env, "/dev/sdz", "--yes")
    assert p.returncode == 0, p.stdout + p.stderr
    again = calls.read_text().splitlines()[n:]
    assert all(x.startswith("mount") for x in again), again  # nothing destructive the second time
    assert "kept /data/home/joe" in p.stdout and "all lines already there" in p.stdout
    assert (d / "home/joe/.config/kilo/env").read_text() == "changed on the Pi\n"
    assert (f / "sdz2/etc/fstab").read_text() == fstab
