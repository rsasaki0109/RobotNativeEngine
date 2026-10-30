import copy
import os
from pathlib import Path
import unittest
import force_friction as p


class ForceFrictionTests(unittest.TestCase):
    def test_mujoco_uses_matched_material_and_normal_gravity(self):
        c = p.cases()[0]
        model = p.mujoco.MjModel.from_xml_string(p.model_xml(c))
        self.assertEqual(model.opt.timestep, 0.002)
        self.assertEqual(list(model.opt.gravity), [0, -9.81, 0])
        self.assertEqual(model.body_mass[1], c['mass_kg'])
        self.assertEqual(model.geom_friction[1, 0], float(p.np.float32(c['mu'])))
        self.assertEqual(list(model.geom_friction[1, 1:]), [0, 0])
        self.assertEqual(model.geom_condim[1], 3)

    def test_mujoco_force_replay_and_timing_state(self):
        c = p.cases()[2]
        r = p.measure(p.run_mujoco, c)
        self.assertTrue(r['exact_replay'])
        # At this coarse dt the initially unloaded contact violates the gates.
        self.assertFalse(r['metrics']['passed'])
        a = copy.deepcopy(r['audit'])
        a['load_mode'] = 'equivalent_gravity'
        with self.assertRaises(ValueError):
            p.quality(c, a)
        a = copy.deepcopy(r['audit'])
        a['warnings'][0] = 1
        self.assertFalse(p.quality(c, a)['gates']['backend_health'])

    def test_quality_failure_suppresses_cost_ratio(self):
        row = {b: {'metrics': {'passed': True}, 'median_cost': t}
               for b, t in [('native', 2), ('mujoco', 4)]}
        self.assertEqual(p.comparison([row])['mujoco_over_native_cost'], 2)
        row['mujoco']['metrics']['passed'] = False
        self.assertEqual(p.comparison([row]), {'eligible': False, 'mujoco_over_native_cost': None})

    @unittest.skipUnless(os.environ.get('RNE_THRESHOLD'), 'requires native release runner')
    def test_native_actual_force_replay_both_directions(self):
        binary = Path(os.environ['RNE_THRESHOLD']).resolve()
        for ratio in (-1.2, -0.8, 0.8, 1.2):
            c = dict(dt_ticks=2000000, mass_kg=3, mu=0.5, force_ratio=ratio)
            r = p.measure(lambda c, a: p.run_native(binary, c, a), c)
            self.assertTrue(r['metrics']['passed'], r['metrics'])
            a = copy.deepcopy(r['audit'])
            a['debug_assertions'] = True
            self.assertFalse(p.quality(c, a)['gates']['backend_health'])


if __name__ == '__main__':
    unittest.main()
