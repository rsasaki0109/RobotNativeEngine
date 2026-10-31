import copy
import os
from pathlib import Path
import unittest
import native_contact_diagnostics as p


@unittest.skipUnless(os.environ.get('RNE_THRESHOLD'),'requires rebuilt native example')
class NativeContactDiagnosticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary=Path(os.environ['RNE_THRESHOLD']).resolve()
        cls.audit=p.run(cls.binary,True)

    def test_enabled_trace_and_impulses_replay_without_changing_motion(self):
        repeated=p.run(self.binary,True)
        disabled=p.run(self.binary,False)
        self.assertEqual(self.audit['trace'],repeated['trace'])
        self.assertEqual(self.audit['contact_diagnostics'],repeated['contact_diagnostics'])
        self.assertEqual(self.audit['trace'],disabled['trace'][:26])
        self.assertFalse(disabled['contact_diagnostics'])
        result=p.summarize(self.audit)
        self.assertLess(result['max_contact_moment_nm'],5e-6)
        self.assertGreaterEqual(min(result['loaded_contacts']),3)
        self.assertLessEqual(max(result['loaded_contacts']),4)

    def test_corrupt_application_point_is_rejected(self):
        altered=copy.deepcopy(self.audit)
        altered['contact_diagnostics'][0]['points'][0]['point_world_m'][0]+=.01
        with self.assertRaises(ValueError):
            p.summarize(altered)

    def test_corrupt_tangential_impulse_is_rejected(self):
        altered=copy.deepcopy(self.audit)
        altered['contact_diagnostics'][0]['points'][0]['tangential_impulse_world_n_s'][0]+=.01
        with self.assertRaises(ValueError):
            p.summarize(altered)


if __name__=='__main__':
    unittest.main()
