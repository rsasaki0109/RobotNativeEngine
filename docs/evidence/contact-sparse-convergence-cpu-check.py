import json, os, resource, statistics, subprocess, hashlib
from pathlib import Path
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
binaries={'before':Path('/workspace/artifacts/contact-sparse-delassus-after'), 'after':Path('/workspace/artifacts/contact-sparse-convergence-v2')}
rows={k:[] for k in binaries}
def run(path):
 u=resource.getrusage(resource.RUSAGE_CHILDREN)
 sample=json.loads(subprocess.run([str(path),'64','timing','clusters'],check=True,capture_output=True,text=True).stdout)
 v=resource.getrusage(resource.RUSAGE_CHILDREN)
 return {'process_cpu_s':v.ru_utime+v.ru_stime-u.ru_utime-u.ru_stime,'sample':sample}
for p in binaries.values():run(p)
for i in range(9):
 for name in (['before','after'] if i%2==0 else ['after','before']):rows[name].append(run(binaries[name]))
assert len({r['sample']['final_hash'] for rs in rows.values() for r in rs})==1
medians={k:{'process_cpu_s':statistics.median(r['process_cpu_s'] for r in rs),'step_wall_s':statistics.median(r['sample']['elapsed_s'] for r in rs)} for k,rs in rows.items()}
r={'protocol':'external_process_cpu_diagnostic_v1','scope':'CPU includes startup, initialization, stepping and output; step wall excludes setup/output. Not interchangeable cost measures.','cpu_affinity':sorted(os.sched_getaffinity(0)),'binary_sha256':{k:hashlib.sha256(p.read_bytes()).hexdigest() for k,p in binaries.items()},'samples':rows,'medians':medians}
Path('/workspace/artifacts/sparse-convergence-v2-cpu64.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps(medians),flush=True)
