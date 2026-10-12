"""Pure whole-byte retention fault probes, with only owned synthetic files."""
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest
import uuid

OWN=Path(__file__).resolve().parent

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value

EXPORT=module('literal_export',OWN/'prepare_literal_export.py')
VERIFY=module('literal_verify',OWN/'verify_literals.py')

class WholeByteGuards(unittest.TestCase):
    def setUp(self):
        self.root=OWN/'SYNTHETIC-EXPORT-FAULT-PROBES'/str(uuid.uuid4());self.root.mkdir(parents=True)
        self.input=self.root/'whole.bin';self.input.write_bytes(b'whole\x00opaque\r\nbytes\xff')
        self.row={**EXPORT.descriptor(self.input),'export_path':'f/000001.bin'}

    def plan(self,rows):
        path=self.root/'plan.json'
        path.write_bytes(EXPORT.canonical({'kind':'rne_motor_construction_literal_export_plan',
            'status':'PREPARATION_ONLY_SIX_POSTCOLLECTION_CLOSURE_PENDING_NOT_PUBLISHED',
            'literal_payloads':rows,'generated_metadata':[]}))
        return path,EXPORT.descriptor(path)['sha256']

    def test_stale_hash_rejected_before_destination_creation(self):
        p,sha=self.plan([self.row]);self.input.write_bytes(b'changed')
        out=self.root/'out'
        with self.assertRaisesRegex(RuntimeError,'stale input'):EXPORT.copy_literals(p,sha,out)
        self.assertFalse(out.exists())

    def test_parent_traversal_and_casefold_collisions_rejected(self):
        row={**self.row,'export_path':'../escaped.bin'};p,sha=self.plan([row])
        with self.assertRaisesRegex(RuntimeError,'unsafe'):EXPORT.copy_literals(p,sha,self.root/'out')
        p.unlink()  # Only this test's generated prospective plan, not an original.
        p,sha=self.plan([self.row,{**self.row,'export_path':'F/000001.BIN'}])
        with self.assertRaisesRegex(RuntimeError,'casefold'):EXPORT.copy_literals(p,sha,self.root/'out')
        self.assertFalse((self.root/'out').exists())

    def test_large_whole_payload_not_split_or_copied(self):
        p,sha=self.plan([{**self.row,'size_bytes':(1<<20)+1}])
        with self.assertRaisesRegex(RuntimeError,'inline file exceeds'):EXPORT.copy_literals(p,sha,self.root/'out')
        self.assertFalse((self.root/'out').exists())

    def test_whole_bytes_and_aliases_verified_then_corruption_rejected(self):
        p,sha=self.plan([self.row]);out=self.root/'out';EXPORT.copy_literals(p,sha,out)
        self.assertEqual((out/'f/000001.bin').read_bytes(),self.input.read_bytes())
        (out/'.gitattributes').write_bytes(b'* -text\n')
        row=[str(self.input),self.row['sha256'],self.row['size_bytes'],self.row['export_path']]
        (out/'origin-map.json').write_text(json.dumps({'kind':'rne_literal_export_origin_map','rows':[row,[str(self.root/'other-origin.bin'),*row[1:]]]}))
        result=VERIFY.verify(out)
        self.assertEqual(result['unique_payloads'],1);self.assertEqual(result['origins'],2)
        (out/'f/000001.bin').write_bytes(b'corrupt')
        with self.assertRaisesRegex(ValueError,'SHA/size mismatch'):VERIFY.verify(out)

if __name__=='__main__':unittest.main(verbosity=2)
