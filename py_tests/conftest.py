"""Shared fixtures for the fanuc_ucl Python tests.

Nothing here needs a controller: drivers are pointed at a closed port on
loopback (connection refused at once) or at an address that never answers.
"""

import pathlib
import subprocess
import sys

import pytest

STUB_ROOT = pathlib.Path(__file__).resolve().parent.parent / "py_src" / "fanuc_ucl"

CLOSED = "127.0.0.1"
"""Loopback: nothing listens on the fixed driver ports, so connects are refused."""

UNROUTABLE = "10.255.255.1"
"""Never answers, so a connect attempt would block until its timeout."""


def run_isolated(code: str, timeout: float = 20.0) -> subprocess.CompletedProcess:
    """Runs `code` in a fresh interpreter, for import-order and blocking checks."""
    return subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


@pytest.fixture
def stub_root() -> pathlib.Path:
    return STUB_ROOT
