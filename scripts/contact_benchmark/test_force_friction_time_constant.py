import unittest
import force_friction_time_constant as soft
import force_friction_matched as hard


class TimeConstantTests(unittest.TestCase):
    def test_old_protocol_and_reserved_cases_are_preserved(self):
        self.assertEqual(hard.SOLREFS, [[2e-6,10],[2e-6,1]])
        self.assertEqual(hard.TIMESTEPS, [2000000,250000,31250,8000,5000])
        self.assertEqual(soft.VALIDATION,hard.VALIDATION)
        self.assertNotEqual(soft.PROTOCOL,hard.PROTOCOL)
        self.assertTrue(all(ref[0] > 2*max(soft.TIMESTEPS)*1e-9 for ref in soft.SOLREFS))

    def test_wider_time_constant_keeps_safety_flags_and_fixture(self):
        c=soft.case(soft.CALIBRATION[1],8000)
        for ref in soft.SOLREFS:
            model=soft.mujoco.MjModel.from_xml_string(soft.model_xml(c,ref))
            self.assertEqual(list(model.geom_solref[1]),ref)
            self.assertEqual(model.opt.disableflags,0)
            self.assertEqual(list(model.opt.gravity),[0,-9.81,0])
            self.assertEqual(model.body_mass[1],3)
            self.assertEqual(model.geom_friction[1,0],0.5)
            self.assertEqual(model.geom_condim[1],3)


if __name__ == '__main__':
    unittest.main()
