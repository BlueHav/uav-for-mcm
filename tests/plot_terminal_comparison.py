#!/usr/bin/env python3
"""Server-only plots of measured speed histories and final physical star slots."""
import argparse,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from point_mission_core import PointMission


def load(run):
    paths=list((Path(run)/'Reports').rglob('telemetry.jsonl'))
    if len(paths)!=1:raise ValueError('Expected one telemetry file')
    series={i:[] for i in range(10)};first=None;last={}
    for line in paths[0].open():
        row=json.loads(line)
        if row['status'].get('planner_mode')!='YOPO':continue
        stamp=row['status']['stamp']
        if first is None:first=stamp
        for key,s in row['states'].items():
            i=int(key);last[i]=s
            if s['phase'] in ('TERRAIN_FOLLOW','END_HOLD'):
                series[i].append((stamp-first,math.hypot(*s['velocity'][:2])))
    return series,last


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--baseline',required=True);ap.add_argument('--run',required=True);ap.add_argument('--out',required=True)
    args=ap.parse_args();out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    old,_=load(args.baseline);new,last=load(args.run)
    fig,axes=plt.subplots(5,2,figsize=(13,11),sharex=True,sharey=True,layout='constrained')
    for i,ax in enumerate(axes.flat):
        for data,label,color in ((old,'V1','#999999'),(new,'V2','#1479b8')):
            xy=np.array(data[i]);ax.plot(xy[:,0],xy[:,1],lw=.7,color=color,alpha=.85,label=label)
        ax.set_title('Drone %d'%i,fontsize=10);ax.set_ylim(-.1,7.5);ax.grid(alpha=.15)
        if i%2==0:ax.set_ylabel('XY speed (m/s)')
        if i>=8:ax.set_xlabel('Seconds since YOPO handover')
    axes[0,0].legend(ncol=2,fontsize=9)
    fig.suptitle('Measured terminal flight speeds — includes intended endpoint deceleration')
    fig.savefig(out/'terminal-speeds.png',dpi=180);plt.close(fig)
    model=PointMission(Path(__file__).resolve().parents[1]/'config/point.json')
    fig,axes=plt.subplots(1,2,figsize=(10,5),layout='constrained')
    for g,ax in enumerate(axes):
        origin=model.ends[g];forward=origin-model.assembly_anchor(g);forward/=np.linalg.norm(forward)
        rotation=np.array([[forward[1],-forward[0]],forward]).T
        target=(model.landing_slots(g)[:,:2]-origin)@rotation
        actual=(np.array([last[5*g+i]['position'][:2] for i in range(5)])-origin)@rotation
        for a,b in ((2,0),(2,1),(0,3),(1,4)):
            ax.plot(target[[a,b],0],target[[a,b],1],color='#aaaaaa',lw=1)
        ax.scatter(target[:,0],target[:,1],s=95,facecolors='none',edgecolors='#555555',marker='D',label='Target star')
        ax.scatter(actual[:,0],actual[:,1],s=45,color='#1479b8',label='Actual touchdown')
        ax.scatter(actual[2,0],actual[2,1],s=60,color='#d65f3b',marker='^',label='Leader')
        for i,xy in enumerate(actual):ax.annotate(str(5*g+i),xy,xytext=(7,6),textcoords='offset points')
        ax.set_title('Group %d — leader %d'%(g,5*g+2));ax.set_aspect('equal');ax.set_xlim(-8,8);ax.set_ylim(-9,3)
        ax.set_xlabel('Lateral offset (m)');ax.set_ylabel('Forward offset (m)');ax.grid(alpha=.15)
    axes[0].legend(fontsize=8,loc='lower left');fig.savefig(out/'star-touchdown.png',dpi=180);plt.close(fig)

if __name__=='__main__':main()
