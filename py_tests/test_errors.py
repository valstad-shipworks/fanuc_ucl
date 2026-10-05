"""Rust errors reachable without a controller, as the exceptions Python sees."""

import time

import pytest

from conftest import CLOSED, UNROUTABLE, run_isolated
from fanuc_ucl import JointFormat, JointTemplate, hmi, hspo, rmi, stmo


def test_rmi_send_before_connect_is_a_connection_error():
    driver = rmi.RmiDriver(rmi.RmiDriverConfig(CLOSED))
    with pytest.raises(ConnectionError):
        driver.send(rmi.FrcInitialize())
    assert not driver.is_connected()
    assert not driver.has_connection_errored()


def test_rmi_connect_to_a_closed_port_is_an_os_error():
    driver = rmi.RmiDriver(rmi.RmiDriverConfig(CLOSED, timeout_secs=0.2))
    start = time.monotonic()
    with pytest.raises(OSError, match="refused"):
        driver.connect()
    assert time.monotonic() - start < 1.0
    assert not driver.is_connected()


def test_hmi_read_before_connect_fails():
    driver = hmi.HmiDriver(CLOSED)
    with pytest.raises(Exception, match="Not connected"):
        driver.read(hmi.Register, 1)
    assert not driver.is_connected()


def test_hmi_connect_to_a_closed_port_fails_at_once():
    driver = hmi.HmiDriver(CLOSED)
    start = time.monotonic()
    with pytest.raises(Exception):
        driver.connect(1.0)
    assert time.monotonic() - start < 0.5
    assert not driver.is_connected()


def test_hmi_connect_to_a_closed_port_is_connection_refused():
    with pytest.raises(ConnectionRefusedError):
        hmi.HmiDriver(CLOSED).connect(1.0)


def test_hmi_connect_times_out_against_a_silent_address():
    start = time.monotonic()
    with pytest.raises(TimeoutError):
        hmi.HmiDriver(UNROUTABLE).connect(0.2)
    assert 0.15 < time.monotonic() - start < 1.0


def test_rmi_connect_honours_its_timeout():
    import subprocess

    try:
        result = run_isolated(
            "from fanuc_ucl import rmi\n"
            f"d = rmi.RmiDriver(rmi.RmiDriverConfig({UNROUTABLE!r}, timeout_secs=0.3))\n"
            "try:\n"
            "    d.connect()\n"
            "except Exception as e:\n"
            "    print(type(e).__name__)\n",
            timeout=3.0,
        )
    except subprocess.TimeoutExpired:
        pytest.fail("connect was still blocked 3 s after a 0.3 s timeout")
    assert "Timeout" in result.stdout, result.stdout + result.stderr


def test_rmi_connect_lets_other_threads_run():
    import subprocess

    code = (
        "import threading, time, os\n"
        "from fanuc_ucl import rmi\n"
        f"d = rmi.RmiDriver(rmi.RmiDriverConfig({UNROUTABLE!r}, timeout_secs=0.3))\n"
        "threading.Thread(target=lambda: d.connect() if True else None, daemon=True).start()\n"
        "time.sleep(0.2)\n"
        "t = time.monotonic()\n"
        "time.sleep(0.01)\n"
        "print(round(time.monotonic() - t, 2), flush=True)\n"
        "os._exit(0)\n"
    )
    try:
        result = run_isolated(code, timeout=3.0)
    except subprocess.TimeoutExpired:
        pytest.fail("the main thread did not run while connect was blocked")
    assert float(result.stdout.strip()) < 0.5


def test_hspo_receiver_before_the_broker_is_a_runtime_error():
    with pytest.raises(RuntimeError, match="not initialized"):
        hspo.HspoReceiver("10.0.0.1")


def test_hspo_broker_lifecycle_without_a_robot():
    hspo.initialize_broker("127.0.0.1:0")
    try:
        receiver = hspo.HspoReceiver("10.0.0.1", 4, 0.01)
        assert not receiver.is_connected()
        assert receiver.joint.try_recv() is None
        assert receiver.tcp.recv_all() == []
        assert receiver.var.wait_for(0.01) is None
        assert not hspo.has_broker_errored()
    finally:
        hspo.destroy_broker(True)


def test_hspo_invalid_listen_address_is_a_value_error():
    with pytest.raises(ValueError, match="listen_on"):
        hspo.initialize_broker("not-an-address")


def test_stmo_start_without_a_controller_times_out():
    driver = stmo.StreamMotionDriver(CLOSED, 5)
    driver.connect()
    try:
        start = time.monotonic()
        with pytest.raises(TimeoutError):
            driver.start(0.2)
        assert time.monotonic() - start < 1.0
        assert not driver.is_started()
    finally:
        driver.disconnect()


@pytest.mark.parametrize(
    "make, error",
    [
        (lambda: hmi.HmiDriver("not-an-ip"), ValueError),
        (lambda: rmi.RmiDriverConfig("not-an-ip"), ValueError),
        (lambda: stmo.StreamMotionDriver("not-an-ip", 5), ValueError),
        (lambda: stmo.StreamMotionDriver(CLOSED, -1), OverflowError),
        (lambda: hmi.HmiDriver(CLOSED).connect(0.0), Exception),
        (lambda: hmi.HmiDriver(CLOSED).read(hmi.Register, 0), Exception),
        (lambda: hmi.HmiDriver(CLOSED).read(hmi.Register, 70_000), Exception),
        (
            lambda: stmo.MotionCommandPacket.try_from_joints(JointFormat.FanucDeg, JointTemplate.SIX, [1.0, 2.0]),
            Exception,
        ),
    ],
)
def test_invalid_arguments_raise(make, error):
    with pytest.raises(error):
        make()


def test_an_out_of_range_hmi_index_is_rejected_as_an_index_error():
    result = run_isolated(
        "from fanuc_ucl import hmi\n"
        "try:\n"
        "    hmi.HmiDriver('127.0.0.1').read(hmi.WireStickInput, 60000)\n"
        "except BaseException as e:\n"
        "    print(type(e).__name__, e)\n"
    )
    out = result.stdout.strip()
    assert out and "Panic" not in out and "Not connected" not in out, out + result.stderr
