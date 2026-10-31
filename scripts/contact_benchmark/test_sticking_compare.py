import os
from pathlib import Path
import subprocess
import unittest
import contact_sticking_compare as s
import test_stopping_compare


class StickingCompareTests(unittest.TestCase):
    def test_comparison_requires_explicit_enabled_mode(self):
        r=test_stopping_compare.StoppingCompareTests().row()
        with self.assertRaises(ValueError):s.summary(r)
        for a in [r['after']]+r['timing']['after']:a['experimental_sticking_block']=True
        self.assertTrue(s.summary(r)['quality']['after']['passed'])
        r['after']['contact_frames'][20]['all_solver_stopping_flags']=False
        self.assertFalse(s.summary(r)['quality']['after']['passed'])

    def test_release_mode_and_replay(self):
        binary=Path(os.environ.get('RNE_CROWD','target/release/examples/contact_crowd'))
        if not binary.is_file():self.skipTest('build release contact_crowd')
        a=s.run(binary,4,'tower-slide','audit',False)
        b=s.run(binary,4,'tower-slide','audit',True)
        self.assertNotIn('experimental_sticking_block',a)
        self.assertIs(b['experimental_sticking_block'],True)
        s.tower.equal_audits(b,s.run(binary,4,'tower-slide','audit',True))
        self.assertEqual(s.run(binary,4,'tower-slide','timing',True)['final_hash'],b['final_hash'])
        bad=subprocess.run([str(binary.resolve()),'4','audit','tower-slide','1','unknown'],
                           capture_output=True,text=True,timeout=120)
        self.assertNotEqual(bad.returncode,0)


if __name__=='__main__':unittest.main()
