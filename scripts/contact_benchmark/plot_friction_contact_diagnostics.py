"""Inspect the support/moment transition around the first angular-speed failure."""
import argparse
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args()
    report=json.loads((args.directory/'report.json').read_text())
    row=report['results'][0]
    path=args.directory/row['file']
    if hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:
        raise ValueError('trace hash differs')
    with np.load(path,allow_pickle=False) as data:
        s=data['steps'];p=data['points']
    index=p[:,0].astype(int)
    lever=p[:,4:7]-s[index,4:7]
    normal=np.cross(lever,p[:,16,None]*p[:,7:10])
    normal_z=np.zeros(len(s));np.add.at(normal_z,index,normal[:,2])
    fig,axes=plt.subplots(3,1,figsize=(9,7),constrained_layout=True)
    gate=row['summary']['first_spin_gate_time_s']
    window=(s[:,0]>=gate-.001)&(s[:,0]<=gate+.001)
    t=s[:,0]*1000
    axes[0].step(t[window],s[window,3],where='post',label='Loaded points (> 1e-8 N)')
    axes[0].set(ylabel='Loaded contacts',yticks=[0,2,4],title='MuJoCo: 20 ms time constant, 8 us step, zero margin')
    axes[1].plot(t[window],normal_z[window],label='Normal moment Z')
    axes[1].plot(t[window],s[window,12]-normal_z[window],label='Tangential moment Z')
    axes[1].plot(t[window],s[window,12],label='Net moment Z',color='black')
    axes[1].set(ylabel='Moment (N m)')
    axes[2].plot(s[:,1]*1000,np.linalg.norm(s[:,28:31],axis=1),label='MuJoCo angular speed')
    native=np.asarray(report['native']['trace'])
    axes[2].plot(native[:,0]*1000,np.linalg.norm(native[:,11:14],axis=1),'o-',label='Native ECS angular speed (2 ms)')
    axes[2].axhline(.01,color='black',linestyle=':',label='Quality gate')
    axes[2].set(ylabel='Angular speed (rad/s)')
    for ax in axes:
        ax.axvline(gate*1000,color='red',linestyle='--',linewidth=1,label='First gate crossing')
        ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_xlabel('Time (ms)')
    axes[0].set_xlabel('Step start (ms)')
    axes[1].set_xlabel('Step start (ms)')
    axes[2].set_xlabel('State time at step end (ms)')
    fig.savefig(args.output,dpi=150)
    plt.close(fig)


if __name__=='__main__':
    main()
