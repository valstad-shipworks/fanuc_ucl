"""The `.pyi` stubs describe the extension module that is actually built:
every public class and function exists, with the same parameter names and
the same parameters defaulted."""

import ast
import importlib
import inspect

import pytest
from conftest import STUB_ROOT, run_isolated

MODULES = {
    "__init__.pyi": "fanuc_ucl",
    "_common.pyi": "fanuc_ucl",
    "hmi/__init__.pyi": "fanuc_ucl.hmi",
    "hmi/asg.pyi": "fanuc_ucl.hmi.asg",
    "hspo/__init__.pyi": "fanuc_ucl.hspo",
    "rmi/__init__.pyi": "fanuc_ucl.rmi",
    "rmi/proto.pyi": "fanuc_ucl.rmi.proto",
    "stmo/__init__.pyi": "fanuc_ucl.stmo",
}

TYPING_ONLY_BASES = {"Protocol", "TypedDict", "Generic"}


def _decorators(node) -> set[str]:
    names = set()
    for d in node.decorator_list:
        while isinstance(d, ast.Call):
            d = d.func
        if isinstance(d, ast.Attribute):
            names.add(d.attr)
        elif isinstance(d, ast.Name):
            names.add(d.id)
    return names


def _base_names(node: ast.ClassDef) -> set[str]:
    names = set()
    for b in node.bases:
        while isinstance(b, ast.Subscript):
            b = b.value
        if isinstance(b, ast.Attribute):
            names.add(b.attr)
        elif isinstance(b, ast.Name):
            names.add(b.id)
    return names


def _is_typing_only(node: ast.ClassDef) -> bool:
    bases = _base_names(node)
    return (
        "type_check_only" in _decorators(node)
        or bool(bases & {"Protocol", "TypedDict"})
        or node.name.startswith("__")
    )


def _params(fn: ast.FunctionDef, method: bool) -> list[tuple[str, bool]]:
    args = fn.args
    positional = args.posonlyargs + args.args
    defaults = [False] * (len(positional) - len(args.defaults)) + [True] * len(
        args.defaults
    )
    params = list(zip([a.arg for a in positional], defaults))
    if method and "staticmethod" not in _decorators(fn) and params:
        params = params[1:]
    params += [
        (a.arg, d is not None) for a, d in zip(args.kwonlyargs, args.kw_defaults)
    ]
    return params


def _runtime_params(obj, method: bool) -> list[tuple[str, bool]] | None:
    try:
        sig = inspect.signature(obj)
    except (TypeError, ValueError):
        return None
    params = [
        p
        for p in sig.parameters.values()
        if p.kind not in (p.VAR_POSITIONAL, p.VAR_KEYWORD)
    ]
    if any(
        p.kind in (p.VAR_POSITIONAL, p.VAR_KEYWORD) for p in sig.parameters.values()
    ):
        return None
    if method and params and params[0].name in ("self", "cls", "$self", "$cls"):
        params = params[1:]
    return [(p.name, p.default is not inspect.Parameter.empty) for p in params]


def _mismatches(stub_file: str) -> list[str]:
    tree = ast.parse((STUB_ROOT / stub_file).read_text())
    module = importlib.import_module(MODULES[stub_file])
    problems = []

    def check_function(fn: ast.FunctionDef, runtime, where: str, method: bool) -> None:
        if fn.name.startswith("_") and fn.name != "__init__":
            return
        decorators = _decorators(fn)
        if "overload" in decorators:
            return
        if fn.name == "__init__":
            target = runtime
        else:
            if not hasattr(runtime, fn.name):
                problems.append(f"{where}.{fn.name} missing at runtime")
                return
            target = getattr(runtime, fn.name)
        if "property" in decorators:
            return
        if inspect.isdatadescriptor(target) and fn.name != "__init__":
            problems.append(
                f"{where}.{fn.name} is a method in the stub but an attribute at runtime"
            )
            return
        got = _runtime_params(target, method and fn.name != "__init__")
        want = _params(fn, method)
        if got is not None and got != want:
            problems.append(f"{where}.{fn.name}: stub {want} != runtime {got}")

    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and not node.name.startswith("_"):
            check_function(node, module, MODULES[stub_file], method=False)
        elif isinstance(node, ast.ClassDef) and not node.name.startswith("_"):
            if _is_typing_only(node):
                continue
            where = f"{MODULES[stub_file]}.{node.name}"
            if not hasattr(module, node.name):
                problems.append(f"{where} missing at runtime")
                continue
            cls = getattr(module, node.name)
            is_enum = bool(_base_names(node) & {"Enum", "IntEnum"})
            for item in node.body:
                if isinstance(item, ast.FunctionDef):
                    check_function(item, cls, where, method=True)
                elif is_enum and isinstance(item, ast.Assign):
                    for t in item.targets:
                        if isinstance(t, ast.Name) and not hasattr(cls, t.id):
                            problems.append(
                                f"{where}.{t.id} enum member missing at runtime"
                            )
    return problems


KNOWN: dict[str, str] = {}


@pytest.mark.parametrize(
    "stub_file",
    [
        pytest.param(f, marks=pytest.mark.xfail(strict=True, reason=KNOWN[f]))
        if f in KNOWN
        else f
        for f in MODULES
    ],
)
def test_stub_matches_runtime(stub_file):
    assert _mismatches(stub_file) == []


def test_every_stubbed_module_imports():
    for name in set(MODULES.values()):
        importlib.import_module(name)


@pytest.mark.xfail(
    strict=True,
    reason="stubs import from `_communication.fanuc_ucl.py_src.fanuc_ucl` / `.....fanuc_ucl.py_src` instead of the package",
)
def test_stub_imports_stay_inside_the_package():
    bad = []
    for path in STUB_ROOT.rglob("*.pyi"):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.ImportFrom):
                target = node.module or ""
                if (
                    node.level > path.relative_to(STUB_ROOT).parts.__len__()
                    or "py_src" in target
                ):
                    bad.append(
                        f"{path.relative_to(STUB_ROOT)}: from {'.' * node.level}{target}"
                    )
    assert bad == []


def test_both_import_styles_give_the_same_module():
    result = run_isolated(
        "from fanuc_ucl import stmo as a\n"
        "import fanuc_ucl.stmo\n"
        "import sys\n"
        "print(a is sys.modules['fanuc_ucl.stmo'])\n"
    )
    assert result.stdout.strip() == "True", result.stderr


def test_version_is_the_installed_distribution():
    from importlib.metadata import version

    import fanuc_ucl

    assert fanuc_ucl.__version__ == version("fanuc_ucl")
