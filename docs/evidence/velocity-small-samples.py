import sys, os, json, statistics
from pathlib import Path
sys.path.insert(0, 'scripts/contact_benchmark')
import contact_crowd as c
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
bins={'before':Path('/workspace/artifacts/contact-clusters-before-sticking-cache'),'after':Path('/workspace/artifacts/contact-clusters-velocity-small')}
for batch in ['a','b']:
 rows={k:[] for k in bins}
 audits={k:c.run(b,1,'audit') for k,b in bins.items()}
 c.check_audits(audits['before'],audits['after'],1)
 for b in bins.values(): c.run(b,1,'timing')
 for i in range(51):
  for name in (['before','after'] if (i+(batch=='b'))%2==0 else ['after','before']):
   rows[name].append(c.run(bins[name],1,'timing'))
 med={k:statistics.median(r['elapsed_s'] for r in v) for k,v in rows.items()}
 r={'protocol':'one_box_followup_51_alternating_samples','batch':batch,'first':'before' if batch=='a' else 'after','cpu_affinity':sorted(os.sched_getaffinity(0)),'binary_sha256':{k:c.digest(b) for k,b in bins.items()},'audit':audits,'timing':rows,'median_elapsed_s':med,'reduction_fraction':1-med['after']/med['before']}
 assert len({x['final_hash'] for rs in rows.values() for x in rs}|{x['final_hash'] for x in audits.values()})==1
 Path('/workspace/artifacts/velocity-small-one-'+batch+'.json').write_text(json.dumps(r,indent=2)+'\n')
 print(batch,med,r['reduction_fraction'],flush=True)
