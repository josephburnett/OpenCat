"""HUPCL handling: every serial open clears HUPCL so DTR stays asserted
between opens (on the NyBoard a DTR edge resets the 328P)."""

import os
import sys
import types

import pytest

from kilo.link import transport as tr


class FakeTermios:
    HUPCL = 0o2000
    TCSANOW = 0
    error = OSError

    def __init__(self, cflag, fail=False):
        self.attrs = [0, 0, cflag, 0, 0, 0, []]
        self.fail = fail
        self.sets = []

    def tcgetattr(self, fd):
        if self.fail:
            raise self.error(25, "Inappropriate ioctl for device")
        return list(self.attrs)

    def tcsetattr(self, fd, when, attrs):
        self.sets.append((fd, when))
        self.attrs = list(attrs)


def test_clear_hupcl_fake_termios():
    t = FakeTermios(cflag=0o2000 | 0o60)
    assert tr.clear_hupcl(7, t)
    assert t.attrs[2] == 0o60 and t.sets == [(7, t.TCSANOW)]
    t = FakeTermios(cflag=0o60)  # already clear: no write
    assert tr.clear_hupcl(7, t) and t.sets == []
    assert not tr.clear_hupcl(7, FakeTermios(cflag=0o2000, fail=True))  # not a tty


@pytest.mark.skipif(sys.platform == "win32", reason="POSIX termios")
def test_clear_hupcl_real_pty_and_non_tty(tmp_path):
    import pty
    import termios

    master, slave = pty.openpty()
    try:
        a = termios.tcgetattr(slave)
        a[2] |= termios.HUPCL
        termios.tcsetattr(slave, termios.TCSANOW, a)
        assert tr.clear_hupcl(slave)
        assert not termios.tcgetattr(slave)[2] & termios.HUPCL
        # by path, as kilo flash does after avrdude
        a[2] |= termios.HUPCL
        termios.tcsetattr(slave, termios.TCSANOW, a)
        assert tr.clear_hupcl_path(os.ttyname(slave))
        assert not termios.tcgetattr(slave)[2] & termios.HUPCL
    finally:
        os.close(master)
        os.close(slave)
    f = tmp_path / "plain"
    f.write_bytes(b"")
    assert not tr.clear_hupcl_path(str(f))  # not a tty: ignored
    assert not tr.clear_hupcl_path(str(tmp_path / "missing"))


def test_serial_transport_clears_hupcl(monkeypatch):
    opened = []

    class FakeSerial:
        def __init__(self, port, baud, **kw):
            opened.append(port)

        def fileno(self):
            return 42

        def set_low_latency_mode(self, on):
            pass

        def reset_input_buffer(self):
            pass

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=FakeSerial))
    seen = []
    monkeypatch.setattr(tr, "clear_hupcl", lambda fd: seen.append(fd))
    tr.SerialTransport("/dev/ttyACM0")
    assert opened == ["/dev/ttyACM0"] and seen == [42]
