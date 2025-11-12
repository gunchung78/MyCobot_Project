from ament_index_python.packages import get_package_share_directory
import json
import numpy as np
import os
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List

def _to_namespace(obj: Any, keep_dict_keys: List[str] = None, parent_key: str = ""):
    if keep_dict_keys is None:
        keep_dict_keys = ["COLOR_RANGES", "COLOR_BRG_DRAW"]

    # COLOR_RANGES, COLOR_BRG_DRAW는 dict 유지
    if isinstance(obj, dict):
        if parent_key in keep_dict_keys:
            return obj  # ✅ dict 그대로 반환
        return SimpleNamespace(**{k: _to_namespace(v, keep_dict_keys, k) for k, v in obj.items()})
    elif isinstance(obj, list):
        return [_to_namespace(v, keep_dict_keys, parent_key) for v in obj]
    else:
        return obj


def _as_uint8_array(x: List[int]) -> np.ndarray:
    return np.array(x, dtype=np.uint8)


def _postprocess(cfg: Dict[str, Any]) -> Dict[str, Any]:
    # ✅ COLOR_RANGES: HSV 범위를 numpy array로 변환
    if "COLOR_RANGES" in cfg:
        cr_fixed = {}
        for name, ranges in cfg["COLOR_RANGES"].items():
            cr_fixed[name] = [
                (_as_uint8_array(lo), _as_uint8_array(hi)) for lo, hi in ranges
            ]
        cfg["COLOR_RANGES"] = cr_fixed

    # ✅ COLOR_BRG_DRAW: BGR 색상 리스트 -> 튜플
    if "COLOR_BRG_DRAW" in cfg:
        cfg["COLOR_BRG_DRAW"] = {
            k: tuple(v) for k, v in cfg["COLOR_BRG_DRAW"].items()
        }

    return cfg


def load_config(path: str = "config.json"):
    """
    ROS2 호환형 config 로더
    1) MYCOBOT_CONFIG 환경변수
    2) 인자 경로 (절대/상대)
    3) ROS2 패키지 share 디렉토리
    4) 현재 작업 디렉토리
    """
    tried = []

    # 1️⃣ 환경 변수
    env_path = os.getenv("MYCOBOT_CONFIG")
    if env_path and Path(env_path).exists():
        with open(env_path, "r", encoding="utf-8") as f:
            return _to_namespace(_postprocess(json.load(f)))

    # 2️⃣ 명시 경로
    if Path(path).exists():
        with open(path, "r", encoding="utf-8") as f:
            return _to_namespace(_postprocess(json.load(f)))
    else:
        tried.append(str(Path(path).resolve()))

    # 3️⃣ ROS 패키지 share 디렉토리
    try:
        pkg_share = Path(get_package_share_directory("mycobot_320_ctrl"))
        share_cfg = pkg_share / "data" / "config.json"
        if share_cfg.exists():
            print(f"[CONFIG] Loaded from: {share_cfg}")
            with open(share_cfg, "r", encoding="utf-8") as f:
                return _to_namespace(_postprocess(json.load(f)))
        tried.append(str(share_cfg))
    except Exception as e:
        tried.append(f"(ament_index_python error: {e})")

    # 4️⃣ CWD fallback
    cwd_cfg = Path.cwd() / "config.json"
    if cwd_cfg.exists():
        with open(cwd_cfg, "r", encoding="utf-8") as f:
            return _to_namespace(_postprocess(json.load(f)))
    tried.append(str(cwd_cfg))

    raise FileNotFoundError(
        "config.json not found.\nTried:\n  - " + "\n  - ".join(tried) +
        f"\nCWD: {Path.cwd()}"
    )

