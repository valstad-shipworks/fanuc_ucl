"""Thread and socket options from Python: every accepted spelling converts,
bad ones raise TypeError/ValueError naming the problem, and an option the
driver refuses raises before any network I/O.

The refusal check runs after conversion, so pairing a spelling with an option
the driver is known to refuse proves the spelling converted: the error that
comes back is the refusal, not a conversion error.
"""

import dataclasses
import enum
import time
import types

import pytest

from conftest import CLOSED, UNROUTABLE
from fanuc_ucl import hmi, hspo, rmi, stmo


def hmi_connect(thread, socket):
    hmi.HmiDriver(UNROUTABLE).connect(0.2, thread, socket)


def rmi_connect(thread, socket):
    rmi.RmiDriver(rmi.RmiDriverConfig(CLOSED, timeout_secs=0.2)).connect(thread, socket)


def hspo_connect(thread, socket):
    try:
        hspo.initialize_broker("127.0.0.1:0", thread, socket)
    finally:
        hspo.destroy_broker(True)


REFUSING = {
    "hmi": (hmi_connect, ("rt_priority", 80), ("recv_buffer", 1 << 20), "RtPriority(80)", "RecvBuffer(1048576)"),
    "rmi": (rmi_connect, ("rt_priority", 80), ("recv_buffer", 1 << 20), "RtPriority(80)", "RecvBuffer(1048576)"),
    "hspo": (
        hspo_connect,
        ("macos_time_constraint", (1000, 500, 800)),
        ("dscp", 46),
        "MacOsTimeConstraint",
        "Dscp(46)",
    ),
}


@dataclasses.dataclass
class Tagged:
    kind: object
    value: object


class Named(enum.Enum):
    linux_nice = "linux_nice"
    dscp = "dscp"


class ThreadOpt(enum.Enum):
    linux_nice = 0
    prefault_stack = 65536


class SocketOpt(enum.Enum):
    dscp = 10


THREAD_SPELLINGS = {
    "tuple": ("linux_nice", 0),
    "camel case": ("LinuxNice", 0),
    "screaming": ("LINUX_NICE", 0),
    "kebab": ("linux-nice", 0),
    "bare name": "win_disable_power_throttling",
    "one-key dict": {"linux_nice": 0},
    "kind/value dict": {"kind": "linux_nice", "value": 0},
    "type/value dict": {"type": "prefault_stack", "value": 65536},
    "dataclass": Tagged("linux_nice", 0),
    "namespace": types.SimpleNamespace(kind="linux_nice", value=0),
    "namespace by type": types.SimpleNamespace(type="linux_nice", value=0),
    "enum kind": Tagged(Named.linux_nice, 0),
    "enum member": ThreadOpt.prefault_stack,
    "cpu list": ("cpu_affinity", [0, 1]),
    "scheduler tuple": ("unix_scheduler", "other"),
    "scheduler dict": {"unix_scheduler": {"kind": "batch"}},
    "qos": ("macos_qos", "user_interactive"),
}

SOCKET_SPELLINGS = {
    "tuple": ("dscp", 10),
    "one-key dict": {"dscp": 10},
    "kind/value dict": {"kind": "linux_priority", "value": 3},
    "dataclass": Tagged("dscp", 10),
    "namespace": types.SimpleNamespace(kind="dscp", value=10),
    "enum kind": Tagged(Named.dscp, 10),
    "enum member": SocketOpt.dscp,
}


def expect_refusal(call, thread, socket, refused: str):
    start = time.monotonic()
    with pytest.raises(ValueError) as err:
        call(thread, socket)
    assert "does not accept option" in str(err.value) and refused in str(err.value), err.value
    assert time.monotonic() - start < 0.1, "a refused option reached the network"


@pytest.mark.parametrize("driver", REFUSING)
@pytest.mark.parametrize("spelling", THREAD_SPELLINGS)
def test_thread_option_spellings_convert(driver, spelling):
    call, refused_thread, _, refused_name, _ = REFUSING[driver]
    expect_refusal(call, [THREAD_SPELLINGS[spelling], refused_thread], None, refused_name)


@pytest.mark.parametrize("driver", ["hmi", "rmi"])
@pytest.mark.parametrize("spelling", SOCKET_SPELLINGS)
def test_socket_option_spellings_convert(driver, spelling):
    call, _, refused_socket, _, refused_name = REFUSING[driver]
    expect_refusal(call, None, [SOCKET_SPELLINGS[spelling], refused_socket], refused_name)


@pytest.mark.parametrize("driver", ["hmi", "rmi"])
def test_a_bool_socket_option_converts(driver):
    call = REFUSING[driver][0]
    expect_refusal(call, None, ("linux_prefer_busy_poll", True), "LinuxPreferBusyPoll(true)")
    expect_refusal(call, None, {"dont_fragment": False}, "DontFragment(false)")


@pytest.mark.parametrize("driver", REFUSING)
def test_a_lone_refused_option_raises(driver):
    call, refused_thread, refused_socket, thread_name, socket_name = REFUSING[driver]
    expect_refusal(call, refused_thread, None, thread_name)
    expect_refusal(call, None, refused_socket, socket_name)
    expect_refusal(call, {"linux_nice": 0, refused_thread[0]: refused_thread[1]}, None, thread_name)


@pytest.mark.parametrize("driver", ["hmi", "rmi"])
def test_a_thread_config_object_converts(driver):
    call, _, _, refused_name, _ = REFUSING[driver]
    expect_refusal(call, types.SimpleNamespace(priority=80, cpu_affinity=None), None, refused_name)
    expect_refusal(call, {"priority": 80, "cpu_affinity": 0}, None, refused_name)


@pytest.mark.parametrize("driver", REFUSING)
@pytest.mark.parametrize(
    "thread, error, mentions",
    [
        (("no_such_option", 1), ValueError, "no_such_option"),
        ([("linux_nice", 0), ("bogus", 1)], ValueError, "bogus"),
        (("linux_nice", "high"), TypeError, "linux_nice"),
        (("rt_priority", 10**20), ValueError, "rt_priority"),
        (("unix_scheduler", "warp"), ValueError, "warp"),
        (("macos_qos", "turbo"), ValueError, "turbo"),
        (3.5, TypeError, "float"),
        ({"kind": 7, "value": 0}, TypeError, "int"),
    ],
)
def test_invalid_thread_options_raise_naming_the_problem(driver, thread, error, mentions):
    call = REFUSING[driver][0]
    start = time.monotonic()
    with pytest.raises(error) as err:
        call(thread, None)
    assert mentions in str(err.value), err.value
    assert time.monotonic() - start < 0.1


@pytest.mark.parametrize(
    "socket, error, mentions",
    [
        (("nope", 1), ValueError, "nope"),
        (("dscp", "ef"), TypeError, "dscp"),
        (("bind_device", 5), TypeError, "bind_device"),
        ({"dscp": -1}, ValueError, "dscp"),
    ],
)
def test_invalid_socket_options_raise_naming_the_problem(socket, error, mentions):
    with pytest.raises(error) as err:
        hmi_connect(None, socket)
    assert mentions in str(err.value), err.value


def test_stmo_accepts_every_option_spelling():
    # macOS refuses a QoS class on a thread whose scheduling policy was set
    # explicitly, so the combined list leaves QoS out.
    thread = [v for k, v in THREAD_SPELLINGS.items() if k != "qos"]
    driver = stmo.StreamMotionDriver(CLOSED, 5)
    try:
        driver.connect(thread, list(SOCKET_SPELLINGS.values()))
        assert driver.is_connected()
    finally:
        driver.disconnect()


def test_stmo_rejects_an_unknown_option():
    with pytest.raises(ValueError, match="nope"):
        stmo.StreamMotionDriver(CLOSED, 5).connect(("nope", 1), None)


def test_none_and_empty_mean_no_options():
    for thread, socket in [(None, None), ([], []), ({}, {})]:
        driver = stmo.StreamMotionDriver(CLOSED, 5)
        try:
            driver.connect(thread, socket)
        finally:
            driver.disconnect()
