# config_loader.py
# -*- coding: utf-8 -*-
import json
import numpy as np
import os
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List

def _to_namespace(obj: Any):
    """dict/list를 재귀적으로 SimpleNamespace/리스트로 변환"""
    if isinstance(obj, dict):
        return SimpleNamespace(**{k: _to_namespace(v) for k, v in obj.items()})
    elif isinstance(obj, list):
        return [ _to_namespace(v) for v in obj ]
    else:
        return obj

def _as_uint8_array(x: List[int]) -> np.ndarray:
    return np.array(x, dtype=np.uint8)

def _postprocess(cfg: Dict[str, Any]) -> Dict[str, Any]:
    """
    JSON을 로드한 뒤, 기존 코드와의 호환을 위해 몇 가지 형식을 정리:
    - COLOR_RANGES: [[[loHSV],[hiHSV]], ...] -> np.uint8 배열 쌍의 리스트
    - COLOR_BRG_DRAW: [B,G,R] 리스트 -> 튜플(그대로 BGR 색상)
    """
    # COLOR_RANGES
    if "COLOR_RANGES" in cfg:
        cr_fixed = {}
        for name, ranges in cfg["COLOR_RANGES"].items():
            fixed_list = []
            for lo, hi in ranges:
                lo_np = _as_uint8_array(lo)
                hi_np = _as_uint8_array(hi)
                fixed_list.append((lo_np, hi_np))
            cr_fixed[name] = fixed_list
        cfg["COLOR_RANGES"] = cr_fixed

    # COLOR_BRG_DRAW
    if "COLOR_BRG_DRAW" in cfg:
        cbd_fixed = {}
        for name, bgr in cfg["COLOR_BRG_DRAW"].items():
            cbd_fixed[name] = tuple(bgr)  # list -> tuple
        cfg["COLOR_BRG_DRAW"] = cbd_fixed

    return cfg

def load_config(path: str = "config.json"):
    """
    config.json 탐색 우선순위:
      1) 환경변수 MYCOBOT_CONFIG
      2) 인자로 받은 경로(절대경로 또는 CWD 기준 상대경로)
      3) 이 파일 기준: ../config.json  (즉 mycobot_main/config.json)
      4) CWD/config.json
    """
    tried = []
    candidates: list[Path] = []

    # 1) ENV
    env_path = os.getenv("MYCOBOT_CONFIG")
    if env_path:
        candidates.append(Path(env_path))

    # 2) ARG
    if path:
        p = Path(path)
        candidates.append(p)
        if not p.is_absolute():
            candidates.append(Path.cwd() / p)

    # 3) MODULE DIR BASED (가장 중요한 고정 경로)
    here = Path(__file__).resolve()              # .../mycobot_main/src/config_loader.py
    root = here.parent.parent                    # .../mycobot_main
    candidates.append(root / "config.json")      # 루트에 있는 config.json

    # 4) CWD
    candidates.append(Path.cwd() / "config.json")

    for c in candidates:
        tried.append(str(c))
        if c.exists():
            with c.open("r", encoding="utf-8") as f:
                raw = json.load(f)
            return _to_namespace(_postprocess(raw))

    raise FileNotFoundError(
        "config.json not found.\nTried:\n  - " + "\n  - ".join(tried) +
        f"\nCWD: {Path.cwd()}"
    )
