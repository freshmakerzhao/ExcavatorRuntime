"""ONNX Runtime policy wrapper：38维 observation -> 4维归一化动作。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import numpy as np


class OnnxPolicyLoadError(RuntimeError):
    """ONNX policy 无法加载或运行。"""


class OnnxPolicy:
    """最小 ONNX Runtime 推理封装，适配 ML-Agents 导出的连续动作模型。"""

    def __init__(self, model_path: Path, providers: Sequence[str] | None = None) -> None:
        try:
            import onnxruntime as ort
        except ModuleNotFoundError as exc:
            raise OnnxPolicyLoadError(
                "未安装 onnxruntime。请在当前 Python 环境安装后再启动 policy bridge，例如："
                " python3 -m pip install onnxruntime"
            ) from exc

        self.model_path = Path(model_path)
        if not self.model_path.exists():
            raise OnnxPolicyLoadError(f"ONNX模型不存在: {self.model_path}")

        selected_providers = list(providers) if providers else ["CPUExecutionProvider"]
        self.session = ort.InferenceSession(str(self.model_path), providers=selected_providers)
        self.input_infos = list(self.session.get_inputs())
        self.output_infos = list(self.session.get_outputs())
        self.observation_input = self._find_observation_input()

    def run(self, observation: Sequence[float]) -> list[float]:
        """执行一次推理，返回 [boom, stick, bucket, swing]，范围 clamp 到 [-1,1]。"""
        obs = np.asarray(list(observation), dtype=np.float32)
        if obs.shape != (38,):
            raise ValueError(f"ONNX observation 必须是38维，实际为 {obs.shape}")

        feed: dict[str, Any] = {}
        for input_info in self.input_infos:
            if input_info.name == self.observation_input.name:
                feed[input_info.name] = self._reshape_observation(obs, input_info.shape)
            else:
                feed[input_info.name] = self._zero_input(input_info)

        outputs = self.session.run(None, feed)
        return self._extract_action(outputs)

    def _find_observation_input(self) -> Any:
        """寻找承载38维向量观测的 ONNX input。"""
        float_inputs = [info for info in self.input_infos if "float" in str(info.type)]
        for info in float_inputs:
            if any(dim == 38 for dim in info.shape):
                return info
        for info in float_inputs:
            if "obs" in info.name.lower() or "observation" in info.name.lower():
                return info
        if float_inputs:
            return float_inputs[0]
        raise OnnxPolicyLoadError("ONNX模型没有可用的float observation输入")

    @staticmethod
    def _reshape_observation(obs: np.ndarray, shape: Sequence[Any]) -> np.ndarray:
        """按模型输入 rank reshape observation；动态 batch 维用 1。"""
        if not shape or len(shape) == 1:
            return obs
        concrete = [1 if not isinstance(dim, int) or dim <= 0 else int(dim) for dim in shape]
        if 38 in concrete:
            concrete[concrete.index(38)] = 38
        if int(np.prod(concrete)) == 38:
            return obs.reshape(concrete)
        if len(concrete) == 2:
            return obs.reshape(1, 38)
        raise OnnxPolicyLoadError(f"无法把38维observation reshape到模型输入shape={shape}")

    @staticmethod
    def _zero_input(input_info: Any) -> np.ndarray:
        """为非 observation 输入构造零值，例如可选 recurrent/action mask 输入。"""
        dtype = np.float32
        type_text = str(input_info.type)
        if "int64" in type_text:
            dtype = np.int64
        elif "int32" in type_text:
            dtype = np.int32
        elif "bool" in type_text:
            dtype = np.bool_

        shape = [1 if not isinstance(dim, int) or dim <= 0 else int(dim) for dim in input_info.shape]
        if not shape:
            shape = [1]
        return np.zeros(shape, dtype=dtype)

    @staticmethod
    def _extract_action(outputs: Sequence[Any]) -> list[float]:
        """从 ONNX outputs 中找到4维连续动作并裁剪。"""
        arrays = [np.asarray(output) for output in outputs]
        candidates = [array for array in arrays if array.size == 4]
        if not candidates:
            candidates = [array for array in arrays if array.size >= 4 and np.issubdtype(array.dtype, np.number)]
        if not candidates:
            raise OnnxPolicyLoadError("ONNX输出中找不到4维动作")

        action = candidates[0].astype(np.float32).reshape(-1)[:4]
        action = np.clip(action, -1.0, 1.0)
        return [float(value) for value in action]
