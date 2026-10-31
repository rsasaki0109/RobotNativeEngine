import copy
import unittest
import contact_clusters as p


class ContactClusterTests(unittest.TestCase):
    def audit(self):
        return dict(layout='clusters',contact_frames=[dict(
            loaded_dynamic_pairs=p.expected_support_pairs(64,'clusters'),
            pre_step_dynamic_penetration_m=1e-10,all_solver_stopping_flags=True,
            max_solver_iterations=100,generated=256,retained=128) for _ in range(100)])

    def test_top_boxes_have_distinct_supports(self):
        self.assertEqual(p.expected_support_pairs(4,'clusters'),[[1,3],[2,4]])
        self.assertEqual(p.expected_support_pairs(4,'stacks'),[[1,2],[3,4]])
        self.assertEqual(p.contact_metrics(self.audit(),64,'clusters')['final_loaded_dynamic_pairs'],32)

    def test_missing_or_wrong_support_is_rejected(self):
        a=self.audit();a['contact_frames'][-1]['loaded_dynamic_pairs'].pop()
        with self.assertRaises(ValueError):p.contact_metrics(a,64,'clusters')
        a=self.audit();a['contact_frames'][0]['loaded_dynamic_pairs']=[[1,2]]
        with self.assertRaises(ValueError):p.contact_metrics(a,64,'clusters')

    def test_penetration_and_iteration_failures_are_not_hidden(self):
        original=self.audit()
        for key,value in [('pre_step_dynamic_penetration_m',1e-4),
                          ('all_solver_stopping_flags',False),('max_solver_iterations',200)]:
            a=copy.deepcopy(original);a['contact_frames'][10][key]=value
            with self.assertRaises(ValueError):p.contact_metrics(a,64,'clusters')


if __name__=='__main__':
    unittest.main()
