"""Plot saved 8 us calibration traces and export first spin-gate crossing times."""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import box


def plot(directory, output):
    report=json.loads((directory/'report.json').read_text())
    fig,axes=plt.subplots(1,3,figsize=(12,3.6),constrained_layout=True)
    onset=[]
    for row in report['records']:
        if row['phase']!='calibration' or row['case']['dt_ticks']!=8000:
            continue
        path=directory/row['trace_file']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=row['trace_sha256']:
            raise ValueError('trace digest differs')
        with np.load(path,allow_pickle=False) as data:
            x=data['trace']
        label=f"{row['solref'][0]*1000:g} ms"
        if row['case']['force_ratio']<1:
            axes[0].plot(x[::200,0],np.abs(x[::200,1])*1e6,label=label)
        else:
            spin=np.linalg.norm(x[:,11:14],axis=1)
            first=np.flatnonzero(spin>report['gates']['spin_rad_s'])
            onset.append(dict(solref=row['solref'],dt_ticks=8000,
                              first_spin_gate_time_s=float(x[first[0],0]) if len(first) else None))
            early=x[:,0]<=0.05
            axes[1].plot(x[early,0][::10],spin[early][::10],label=label)
            penetration=np.maximum(0,-box.corners(x)[:,:,1].min(axis=1))
            axes[2].plot(x[::200,0],penetration[::200]*1e6,label=label)
    for axis,title,ylabel,gate in zip(axes,
            ['Static drift (force ratio 0.8)','Sliding spin onset (ratio 1.2)','Sliding penetration (ratio 1.2)'],
            ['Displacement (um)','Angular speed (rad/s)','Penetration (um)'],
            [10,0.01,10]):
        axis.set(title=title,xlabel='Time (s)',ylabel=ylabel)
        axis.axhline(gate,color='black',linestyle='--',linewidth=1,label='Quality gate')
        axis.grid(alpha=0.25)
        axis.legend(fontsize=8)
    fig.suptitle('MuJoCo 3.9.0: 8 us step, contact time constant sweep (display subsampled)')
    fig.savefig(output,dpi=160)
    plt.close(fig)
    return onset


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    print(json.dumps(plot(args.directory,args.output),indent=2))
