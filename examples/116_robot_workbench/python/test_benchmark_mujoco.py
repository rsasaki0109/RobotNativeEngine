"""Analytic torque check and shared-model normalization contract."""
from pathlib import Path
import importlib.util
import subprocess
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from benchmark_mujoco import compare, normalize_urdf, run_mujoco, run_rne, trajectory_records
from test_rne_gym import BINARY

ROTOR = '''<robot name="rotor">
<link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
<link name="rotor"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/></inertial>
<collision><geometry><sphere radius="0.1"/></geometry></collision></link>
<joint name="spin" type="revolute"><parent link="base"/><child link="rotor"/><axis xyz="0 0 1"/>
<limit lower="-1" upper="1" effort="2" velocity="5"/><dynamics damping="2" friction="1"/></joint></robot>'''


class TorqueBenchmarkTests(unittest.TestCase):
    def request(self):
        urdf, names = normalize_urdf(ROTOR)
        return {"urdf": urdf, "joint_names": names, "dt_ticks": 1_000_000,
                "torques_nm": [[0.01]] * 100 + [[0.0]] * 100}

    def assert_rotor(self, result, inertia=0.02):
        # I=0.02 kg m², tau=0.01 N m => alpha=0.5 rad/s².
        # Compare to continuous motion, not either engine's integration scheme.
        n = np.arange(201)
        alpha = 0.01 / inertia
        expected_v = alpha * np.minimum(n, 100) * 0.001
        expected_q = np.where(n <= 100, alpha * 0.001**2 * n**2 / 2,
                              alpha * 0.1**2 / 2 + (n - 100) * 0.001 * alpha * 0.1)
        np.testing.assert_allclose(np.asarray(result["dq_rad_s"])[:, 0], expected_v, atol=2e-6, rtol=0)
        # Euler's accumulated position bias is alpha*T*dt/2=25 microradians;
        # allow two additional microradians for f32 state extraction.
        np.testing.assert_allclose(np.asarray(result["q_rad"])[:, 0], expected_q,
                                   atol=alpha * 0.1 * 0.001 / 2 + 2e-6, rtol=0)

    def test_rotated_inertia_and_offset_centre_of_mass(self):
        request = self.request()
        source = ROTOR.replace('<origin xyz="0 0 0"/>',
                               '<origin xyz="0.1 0 0" rpy="0 1.5707963267948966 0"/>')
        source = source.replace('ixx="0.02"', 'ixx="0.04"')
        request["urdf"], _ = normalize_urdf(source)
        # Rotated local Ixx about world Z plus m*r²: 0.04 + 1*0.1².
        self.assert_rotor(run_rne(BINARY, request), inertia=0.05)
        if importlib.util.find_spec("mujoco"):
            self.assert_rotor(run_mujoco(request), inertia=0.05)

    def test_so101_missing_dynamics_means_explicit_zero_loss(self):
        # A single rotor misses legacy damping introduced when assembling a chain.
        source = Path(__file__).resolve().parents[3] / "assets/robots/so101/so101.urdf"
        urdf, names = normalize_urdf(source.read_text())
        root = ET.fromstring(urdf)
        for joint in root.findall("joint"):
            if joint.get("type") == "continuous":
                ET.SubElement(joint, "dynamics", damping="0", friction="0")
        pulse = [0.01 if name == "shoulder_pan" else 0.0 for name in names]
        request = {"urdf": urdf, "joint_names": names, "dt_ticks": 1_000_000,
                   "torques_nm": [pulse] * 100 + [[0.0] * len(names)] * 100}
        implicit = run_rne(BINARY, request)
        explicit = run_rne(BINARY, {**request, "urdf": ET.tostring(root, encoding="unicode")})
        for field in ("q_rad", "dq_rad_s"):
            np.testing.assert_array_equal(implicit[field], explicit[field])

    def test_rne_rejects_wrong_joint_order_and_invalid_torque_matrix(self):
        request = self.request()
        for change in ({"joint_names": ["missing"]}, {"torques_nm": [[3.0]]},
                       {"torques_nm": [[]]}):
            with self.assertRaises(subprocess.CalledProcessError):
                run_rne(BINARY, {**request, **change})

    def test_rne_against_analytic_rotor_and_exact_repeat(self):
        request = self.request()
        first, again = run_rne(BINARY, request), run_rne(BINARY, request)
        self.assert_rotor(first)
        self.assertEqual(first["q_rad"], again["q_rad"])
        self.assertEqual(first["dq_rad_s"], again["dq_rad_s"])

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "optional MuJoCo package not installed")
    def test_mujoco_against_analytic_rotor(self):
        self.assert_rotor(run_mujoco(self.request()))

    def test_normalization_preserves_inertia_and_removes_only_declared_conditions(self):
        normalized, names = normalize_urdf(ROTOR)
        root = ET.fromstring(normalized)
        self.assertEqual(names, ["spin"])
        self.assertEqual(root.find("./link[@name='rotor']/inertial/inertia").attrib,
                         ET.fromstring(ROTOR).find("./link[@name='rotor']/inertial/inertia").attrib)
        self.assertIsNone(root.find(".//collision"))
        self.assertIsNone(root.find(".//dynamics"))
        self.assertIsNone(root.find(".//limit"))
        self.assertEqual(root.find("joint").get("type"), "continuous")

    def test_comparison_and_replay_exclude_only_timings(self):
        a = {"joint_names": ["j"], "q_rad": [[0], [1]], "dq_rad_s": [[0], [2]], "runner_loop_s": 1}
        b = {**a, "q_rad": [[0], [2]], "runner_loop_s": 2}
        self.assertEqual(compare(a, b)["max_abs_difference_rad"], [1])
        with self.assertRaises(ValueError):
            compare(a, {**b, "joint_names": ["other"]})
        self.assertEqual(trajectory_records([{"rne": a, "mujoco": a}]),
                         trajectory_records([{"rne": {**a, "runner_loop_s": 99}, "mujoco": a}]))
        self.assertNotEqual(trajectory_records([{"rne": a, "mujoco": a}]),
                            trajectory_records([{"rne": b, "mujoco": a}]))


if __name__ == "__main__":
    unittest.main()
