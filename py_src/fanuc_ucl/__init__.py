from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("fanuc")
except PackageNotFoundError:
    __version__ = "0.0.0"

from importlib import import_module


def __getattr__(name: str):
    core = import_module(f"{__name__}._fanuc_core")
    if hasattr(core, name):
        return getattr(core, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    core = import_module(f"{__name__}._fanuc_core")
    return sorted(set(globals().keys()) | set(dir(core)))


import fanuc_ucl._fanuc_core  # noqa: E402

fanuc_ucl._fanuc_core.config_logging(fanuc_ucl._fanuc_core.LoggingLevel.Err)
