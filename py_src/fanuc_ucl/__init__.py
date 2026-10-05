from importlib.metadata import PackageNotFoundError, version

from fanuc_ucl import _fanuc_core as _fanuc_core  # type: ignore

try:
    __version__ = version("fanuc_ucl")
except PackageNotFoundError:
    __version__ = "0.0.0"

from importlib import import_module

_SUBPACKAGES = {"hmi", "hspo", "rmi", "stmo"}


def __getattr__(name: str):
    if name in _SUBPACKAGES:
        return import_module(f"{__name__}.{name}")
    core = import_module(f"{__name__}._fanuc_core")
    if hasattr(core, name):
        return getattr(core, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    core = import_module(f"{__name__}._fanuc_core")
    return sorted(set(globals().keys()) | set(dir(core)))
