import json

from kilo.log import JsonlLogger


def test_jsonl_logger_writes_and_rotates(tmp_path):
    p = tmp_path / "k.jsonl"
    lg = JsonlLogger(p, max_bytes=2000, backups=2)
    for i in range(100):
        lg.log("rx", type="TELEMETRY", seq=i, payload=b"\x01\x02")
    lg.close()
    files = sorted(tmp_path.iterdir())
    assert len(files) >= 2
    lines = [json.loads(l) for f in files for l in f.read_text().splitlines()]
    assert all(l["kind"] == "rx" for l in lines)
    assert lines[0]["payload"] == "0102" and "t" in lines[0] and "wall" in lines[0]
