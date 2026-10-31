import types
import unittest

import mujoco
import numpy as np
import mujoco_stack as ms
import stack


class MuJoCoStackTests(unittest.TestCase):
    def test_freejoint_angular_velocity_is_rotated_to_world(self):
        d = types.SimpleNamespace(qpos=np.array([0, 0, 0, np.sqrt(0.5), 0, 0, np.sqrt(0.5)]),
                                  qvel=np.array([0, 0, 0, 1, 0, 0]))
        row = ms.sample(d, 1, 0, [0])[0]
        np.testing.assert_allclose(row[11:14], [0, 1, 0], atol=1e-14)

    def test_model_mass_inertia_and_initial_gap_match_native(self):
        case = {'dt_ticks': 2000000, 'levels': 3, 'top_mass_kg': 10, 'gap_m': 0.005}
        model = mujoco.MjModel.from_xml_string(ms.model_xml(case, {'solref': [0.02, 1]}))
        d = mujoco.MjData(model)
        np.testing.assert_allclose(model.body_mass[1:], [1, 1, 10])
        np.testing.assert_allclose(model.body_inertia[3], np.full(3, 10 * 0.1**2 / 6))
        trace = ms.sample(d, 3, 0, [0, 0, 0])
        np.testing.assert_allclose(np.array(trace)[:, 2], [0.055, 0.16, 0.265])
        self.assertEqual(model.ngeom, 4)

    def test_interface_impulses_and_geometry_produce_real_failure(self):
        case = {'dt_ticks': 2000000, 'levels': 2, 'top_mass_kg': 1, 'gap_m': 0.005}
        result = ms.run(case, {'solref': [0.02, 1]}, True)
        quality = stack.metrics(case, result['trace'])
        self.assertFalse(quality['gates']['max_floor_penetration_m'])
        self.assertLess(quality['max_support_relative_error'], 0.01)


if __name__ == '__main__':
    unittest.main()
