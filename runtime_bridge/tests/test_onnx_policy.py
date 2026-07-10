import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from runtime_bridge.onnx_policy import OnnxPolicy, OnnxPolicyLoadError


class _TensorInfo:
    def __init__(self, name, shape, tensor_type="tensor(float)"):
        self.name = name
        self.shape = shape
        self.type = tensor_type


class _PolicySession:
    def __init__(self, _model_path, providers):
        self.providers = providers
        self.sample_index = 0
        self.inputs = [_TensorInfo("obs_0", ["batch", 38])]
        self.outputs = [
            _TensorInfo("version_number", [1]),
            _TensorInfo("memory_size", [1]),
            _TensorInfo("continuous_actions", ["batch", 4]),
            _TensorInfo("continuous_action_output_shape", [1]),
            _TensorInfo("deterministic_continuous_actions", ["batch", 4]),
        ]

    def get_inputs(self):
        return self.inputs

    def get_outputs(self):
        return self.outputs

    def run(self, output_names, _feed):
        self.sample_index += 1
        values = {
            "version_number": np.array([3.0], dtype=np.float32),
            "memory_size": np.array([0.0], dtype=np.float32),
            "continuous_actions": np.array(
                [[0.1 * self.sample_index, -0.1 * self.sample_index, 0.2, -0.2]],
                dtype=np.float32,
            ),
            "continuous_action_output_shape": np.array([4.0], dtype=np.float32),
            "deterministic_continuous_actions": np.array(
                [[0.25, -0.5, 0.75, -1.0]],
                dtype=np.float32,
            ),
        }
        selected_names = self.outputs if output_names is None else output_names
        return [values[item.name if hasattr(item, "name") else item] for item in selected_names]


class _NonFiniteOutputSession(_PolicySession):
    def run(self, _output_names, _feed):
        return [np.array([[0.25, np.nan, 0.75, -1.0]], dtype=np.float32)]


class OnnxPolicyTest(unittest.TestCase):
    @staticmethod
    def _make_policy(session_type=_PolicySession):
        fake_onnxruntime = types.SimpleNamespace(InferenceSession=session_type)
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / "policy.onnx"
            model_path.touch()
            with patch.dict(sys.modules, {"onnxruntime": fake_onnxruntime}):
                return OnnxPolicy(model_path)

    def test_same_observation_uses_stable_deterministic_action_output(self):
        policy = self._make_policy()

        first = policy.run([0.0] * 38)
        second = policy.run([0.0] * 38)

        self.assertEqual(first, [0.25, -0.5, 0.75, -1.0])
        self.assertEqual(second, first)

    def test_rejects_non_finite_observation(self):
        policy = self._make_policy()
        observation = [0.0] * 38
        observation[9] = np.inf

        with self.assertRaisesRegex(ValueError, "非有限"):
            policy.run(observation)

    def test_rejects_non_finite_action_output(self):
        policy = self._make_policy(_NonFiniteOutputSession)

        with self.assertRaisesRegex(OnnxPolicyLoadError, "非有限"):
            policy.run([0.0] * 38)


if __name__ == "__main__":
    unittest.main()
