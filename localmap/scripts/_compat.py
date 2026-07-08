"""兼容旧 scripts 入口的加载工具。"""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import ModuleType


def expose_app(app_path: Path, target_globals: dict) -> ModuleType:
    """加载新的 apps/* 实现，并把公开符号暴露给旧入口模块。"""
    spec = importlib.util.spec_from_file_location(app_path.stem, app_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法加载入口实现: {app_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for name in dir(module):
        if not name.startswith("__"):
            target_globals[name] = getattr(module, name)
    return module


def run_app_main(module: ModuleType) -> int:
    """调用实现模块的main函数；兼容没有显式返回值的入口。"""
    main = getattr(module, "main", None)
    if main is None:
        raise AttributeError(f"{module.__name__} 缺少 main()")
    result = main()
    return 0 if result is None else int(result)
