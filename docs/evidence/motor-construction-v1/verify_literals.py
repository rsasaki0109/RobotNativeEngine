"""Portable byte-only verifier; does not run snapshots, native readers or worlds."""
import argparse
import hashlib
import json
from pathlib import Path
import stat

CAP=1<<20
PREFIX='docs/evidence/motor-construction-v1/'

def require(ok,message):
    if not ok:raise ValueError(message)

def verify(root):
    root=root.resolve()
    require((root/'.gitattributes').read_bytes()==b'* -text\n','literal-byte attributes missing')
    origin=root/'origin-map.json'
    require(origin.stat().st_size<=CAP,'whole origin map over inline limit')
    mapping=json.loads(origin.read_bytes())
    require(mapping['kind']=='rne_literal_export_origin_map','wrong mapping kind')
    origins=set();expected={}
    for source,sha,size,relative in mapping['rows']:
        rel=Path(relative)
        require(relative.isascii() and not rel.is_absolute() and '..' not in rel.parts and str(rel)==relative,'unsafe payload path')
        require(len(PREFIX+relative)<=166,'Windows checkout path too long')
        require(type(size)is int and 0<=size<=CAP and len(sha)==64,'invalid payload descriptor')
        require(source not in origins,'duplicate source origin');origins.add(source)
        key=relative.casefold()
        require(key not in expected or expected[key]==(relative,sha,size),'casefold collision or conflicting alias')
        expected[key]=(relative,sha,size)
    for relative,sha,size in expected.values():
        path=root/relative
        require(path==path.resolve() and stat.S_ISREG(path.stat().st_mode),'symlink/nonregular payload')
        data=path.read_bytes()
        require(len(data)==size and hashlib.sha256(data).hexdigest()==sha,'whole payload SHA/size mismatch')
    for path in root.rglob('*'):
        if path.is_file():require(path.stat().st_size<=CAP,'inline evidence file exceeds limit')
    return {'status':'PASS_LITERAL_EXPORTED_BYTES_ONLY','origins':len(origins),'unique_payloads':len(expected),
            'native_invocations':0,'worlds':0,'numerical_reanalysis':False,
            'qualification_scope':'No scientific outcome or binary availability is inferred from byte verification.'}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,default=Path(__file__).resolve().parent)
    args=p.parse_args();print(json.dumps(verify(args.root),sort_keys=True))
