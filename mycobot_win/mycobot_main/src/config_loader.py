# config_loader.py
# -*- coding: utf-8 -*-
import json
import numpy as np
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
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    fixed = _postprocess(raw)
    # dict -> SimpleNamespace (점 표기 지원)
    return _to_namespace(fixed)
