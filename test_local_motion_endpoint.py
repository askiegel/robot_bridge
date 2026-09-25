import app as bridge

CLEAR = {"ok": True, "reason": None, "age_seconds": .01, "sample_count": 5, "clearance_m": .5}
OBSTACLE = {"ok": True, "reason": None, "age_seconds": .01, "sample_count": 5, "clearance_m": .35}
STALE = {"ok": False, "reason": "lidar_stale", "age_seconds": .31, "sample_count": 0, "clearance_m": None}

class Control:
    def __init__(self, running=False): self.running=running
    def snapshot(self): return {"running": self.running}

def configure(monkeypatch, gates, events=None):
    events = [] if events is None else events
    sequence=iter(gates)
    monkeypatch.setattr(bridge, "forward_clearance", lambda _: events.append("lidar") or next(sequence))
    monkeypatch.setattr(bridge.lidar_telemetry, "snapshot", lambda: {})
    monkeypatch.setattr(bridge, "ros_ready", True); monkeypatch.setattr(bridge, "publisher_node", object())
    for name in ("navigation_control", "mapping_navigation_control", "planning_control", "mapping_control"): monkeypatch.setattr(bridge, name, Control())
    monkeypatch.setattr(bridge, "stop_robot", lambda: events.append("zero") or {"ok": True})
    monkeypatch.setattr(bridge, "acquire_stanford_ownership", lambda: events.append("acquire") or {"ok": True})
    monkeypatch.setattr(bridge, "release_stanford_ownership", lambda: events.append("release") or {"ok": True})
    monkeypatch.setattr(bridge, "clear_streaming_state", lambda: events.append("clear"))
    published=[]
    def publish(x,z): events.append("nonzero" if x else "final_zero"); published.append((x,z)); return {"ok": True}
    monkeypatch.setattr(bridge, "publish_twist", publish)
    clock=[0.0]; monkeypatch.setattr(bridge.time, "monotonic", lambda: clock[0]); monkeypatch.setattr(bridge.time, "sleep", lambda s: clock.__setitem__(0, clock[0]+s))
    bridge.local_motion_active=False
    return events,published

def test_invalid_and_conflict_block_before_acquire(monkeypatch):
    events,published=configure(monkeypatch,[CLEAR])
    assert bridge.app.test_client().post("/local-motion/forward",json={"speed":.2}).status_code==400
    assert events==[] and published==[]
    events,published=configure(monkeypatch,[CLEAR]); bridge.navigation_control.running=True
    response=bridge.app.test_client().post("/local-motion/forward",json={})
    assert response.status_code==409 and response.get_json()["stop_reason"]=="runtime_conflict" and "acquire" not in events

def test_post_owner_recheck_blocks_nonzero_and_cleans(monkeypatch):
    events,published=configure(monkeypatch,[CLEAR,OBSTACLE])
    payload=bridge.app.test_client().post("/local-motion/forward",json={}).get_json()
    assert not payload["executed"] and payload["stop_reason"]=="forward_obstacle"
    assert events.index("zero") < events.index("acquire") < events.index("final_zero") < events.index("release")
    assert not any(x for x,_ in published)

def test_fixed_bounded_motion_and_cleanup(monkeypatch):
    events,published=configure(monkeypatch,[CLEAR]*20)
    payload=bridge.app.test_client().post("/local-motion/forward",json={}).get_json()
    assert payload["executed"] and payload["stop_reason"]=="duration_complete" and payload["elapsed_seconds"]<=.51
    assert all((x,z)==(.1,0.0) for x,z in published if x)
    assert events.index("acquire") < events.index("nonzero") < events.index("final_zero") < events.index("release")

def test_stale_and_publish_failure_release(monkeypatch):
    events,published=configure(monkeypatch,[CLEAR,CLEAR,STALE])
    payload=bridge.app.test_client().post("/local-motion/forward",json={}).get_json()
    assert payload["executed"] and payload["stop_reason"]=="lidar_stale" and "release" in events
    events,published=configure(monkeypatch,[CLEAR,CLEAR])
    monkeypatch.setattr(bridge,"publish_twist",lambda x,z: events.append("final_zero" if not x else "nonzero") or {"ok":not x})
    response=bridge.app.test_client().post("/local-motion/forward",json={})
    assert response.status_code==503 and response.get_json()["stop_reason"]=="motion_publish_failure" and "release" in events

def test_status_is_read_only(monkeypatch):
    events=[]; monkeypatch.setattr(bridge,"forward_clearance",lambda _:CLEAR); monkeypatch.setattr(bridge.lidar_telemetry,"snapshot",lambda:{})
    monkeypatch.setattr(bridge,"publish_twist",lambda *x:events.append("publish")); monkeypatch.setattr(bridge,"acquire_stanford_ownership",lambda:events.append("acquire"))
    assert bridge.app.test_client().get("/local-motion/status").get_json()["start_clear"]
    assert events==[]
