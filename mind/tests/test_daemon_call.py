"""The daemon's generic ``call`` op serves an explicit allow-list
(kilo.daemon.ops.CALL_ALLOWED), not any public BodyClient method."""

import pytest

from kilo.daemon import CALL_ALLOWED, CALL_REFUSED

from test_daemon import remote, sockdir, start_daemon, vbody_pty, wait_for  # noqa: F401 (fixtures)


def test_call_allow_list_is_what_remote_body_needs():
    # RemoteBody reaches these through its __getattr__ proxy (kilo look, ping)
    assert CALL_ALLOWED == {"look", "release_head", "ping"}
    assert not CALL_ALLOWED & CALL_REFUSED


def test_call_op_allow_list(start_daemon):
    d, vb, path, sock = start_daemon()
    body = remote(sock)
    try:
        body.wait_ready(3)
        # allowed: through the RemoteBody proxy -> the call op -> BodyClient
        assert body.ping() > 0
        f = body.look(20, -5)
        assert f["pan"] == 320 and wait_for(lambda: vb.head_until is not None, 2)
        body.release_head()
        assert wait_for(lambda: vb.head_until is None, 2)
        # unknown or not listed (public BodyClient methods included): refused, nothing sent
        epoch = vb.epoch
        for method in ("walking", "reboot", "set_stream", "imu_offsets_set", "trick_done_since",
                       "no_such_method", "_check_drivable", "__init__"):
            with pytest.raises(ValueError, match="not served through `call`"):
                body.conn.call("call", method=method)
        with pytest.raises(ValueError, match="not served through `call`"):
            body.reboot()  # the proxy still exists client-side; the daemon refuses
        assert vb.epoch == epoch and body.conn.call("ping") == "pong"
        # goals and tricks: refused with a pointer to their ops, nothing owned
        for method in ("posture", "set_balance", "walk", "start_trick", "play", "wait_ready"):
            with pytest.raises(ValueError, match="not through `call`; use the posture"):
                body.conn.call("call", method=method, args=["stand"])
        assert not any(o is not None for o in d.goal_owner.values())
    finally:
        body.close()
