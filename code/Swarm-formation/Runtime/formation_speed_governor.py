"""Swarm-Formation graph diagnostics and speed-only, role-ordered regulation.

No trajectory or heading is commanded here. The existing independent YOPO
planner retains obstacle avoidance and all collision/perception protections.
"""
import math
import numpy as np
from formation_core import laplacian,graph_cost,graph_gradient

EDGES=((2,0),(2,1),(0,3),(1,4))

class FormationSpeedGovernor:
    def __init__(self,template,minimum_speed=2.,nominal_speed=6.,coefficient_tolerance=.02,
                 position_tolerance=1.5,lookahead=3.5,axis_turn_rate=.35):
        self.template=np.asarray(template,float)
        values=np.array([minimum_speed,nominal_speed,coefficient_tolerance,position_tolerance,lookahead,axis_turn_rate])
        if self.template.shape!=(5,2) or not np.isfinite(self.template).all() or not np.isfinite(values).all():
            raise ValueError('Invalid formation speed configuration')
        if not 2.<=minimum_speed<=nominal_speed<=6. or min(coefficient_tolerance,position_tolerance,lookahead,axis_turn_rate)<=0:
            raise ValueError('Invalid formation speed limits or gains')
        forward=-np.mean(self.template-self.template[2],axis=0)
        if np.linalg.norm(forward)<1e-6:raise ValueError('Leading star template required')
        self.initial_axis=forward/np.linalg.norm(forward);self.axis=self.initial_axis.copy()
        self.order_course=self.initial_axis.copy()
        right=np.array([self.axis[1],-self.axis[0]])
        relative=self.template-self.template[2]
        self.local=np.c_[relative@right,relative@self.axis]
        self.minimum=float(minimum_speed);self.nominal=float(nominal_speed)
        self.tolerance=float(coefficient_tolerance);self.position_tolerance=float(position_tolerance)
        self.lookahead=float(lookahead);self.turn_rate=float(axis_turn_rate);self.last_stamp=None

    def update(self,positions,velocities,now,forward=None):
        p,v=np.asarray(positions,float),np.asarray(velocities,float);now=float(now)
        if p.shape!=(5,3) or v.shape!=(5,3) or not np.isfinite(np.r_[p.ravel(),v.ravel(),now]).all():
            raise ValueError('Invalid formation observation')
        dt=0. if self.last_stamp is None else min(.25,max(0.,now-self.last_stamp));self.last_stamp=now
        course=v[2,:2] if forward is None else np.asarray(forward,float)
        if course.shape!=(2,) or not np.isfinite(course).all():raise ValueError('Invalid formation course')
        if np.linalg.norm(course)>.5:
            angle=math.atan2(self.axis[0]*course[1]-self.axis[1]*course[0],float(self.axis@course))
            turn=float(np.clip(angle,-self.turn_rate*dt,self.turn_rate*dt))
            c,s=math.cos(turn),math.sin(turn)
            self.axis=np.array([c*self.axis[0]-s*self.axis[1],s*self.axis[0]+c*self.axis[1]])
        # Template alignment is gradual; actual front/back must follow
        # measured travel immediately, including an avoidance turn.
        if np.linalg.norm(v[2,:2])>.5:
            self.order_course=v[2,:2]/np.linalg.norm(v[2,:2])
        right=np.array([self.axis[1],-self.axis[0]])
        offsets=self.local[:,0,None]*right+self.local[:,1,None]*self.axis
        # XY projection uses the exact squared-distance normalized Laplacian
        # coefficient from upstream SwarmGraph::calcFNorm2 (z is zero here).
        actual=np.c_[p[:,:2],np.zeros(5)];desired=np.c_[offsets,np.zeros(5)]
        difference=laplacian(actual)-laplacian(desired)
        rows=np.sum(difference*difference,axis=1);cost=float(rows.sum())
        center=np.median(p[:,:2]-offsets,axis=0);expected=center+offsets
        repairs=[]
        for i in range(5):
            repaired=actual.copy();repaired[i,:2]=expected[i]
            try:improvement=cost-graph_cost(repaired,desired)
            except ValueError:improvement=0.
            repairs.append(max(0.,float(improvement)))
        repairs=np.asarray(repairs);scores=repairs if repairs.max(initial=0.)>1e-8 else rows
        culprits=[int(i) for i in np.argsort(-scores) if scores[i]>1e-8]
        gradient=graph_gradient(actual,desired)[:,:2]
        directions=v[:,:2].copy();length=np.linalg.norm(directions,axis=1)
        directions[length>.2]/=length[length>.2,None];directions[length<=.2]=self.axis
        growth=np.sum(gradient*directions,axis=1)*self.nominal
        caps=np.full(5,self.nominal);reasons=[[] for _ in range(5)]
        if cost>self.tolerance and repairs.max(initial=0.)>1e-8:
            strength=min(1.,(cost-self.tolerance)/.15)
            weights=repairs/max(float(repairs.max()),1e-9)
            for i in range(5):
                if growth[i]<=1e-4:continue
                severity=strength*weights[i]*min(1.,growth[i]/.02)
                caps[i]=max(self.minimum,self.nominal-(self.nominal-self.minimum)*severity)
                if caps[i]<self.nominal-1e-6:reasons[i].append(dict(kind='graph_error_growth',repair_gain=float(repairs[i]),growth_rate=float(growth[i])))
        order=[];offenders=set();recovering=set()
        forward_speed=v[:,:2]@self.order_course
        for parent,child in EDGES:
            gap=float((p[parent,:2]-p[child,:2])@self.order_course)
            desired_gap=float(self.local[parent,1]-self.local[child,1])
            if gap>desired_gap+self.position_tolerance:
                cap=max(self.minimum,self.nominal-(gap-desired_gap-self.position_tolerance))
                caps[parent]=min(caps[parent],cap)
                reasons[parent].append(dict(kind='follower_lag',member=child,gap_m=gap))
            if gap<-.001:
                offenders.add(child);recovering.add(parent);caps[child]=self.minimum
                reasons[child].append(dict(kind='overtaking_predecessor',member=parent,gap_m=gap))
            order.append(dict(parent=parent,child=child,gap_m=gap,desired_gap_m=desired_gap,
                              actual_violation=bool(gap<-.001),parent_below_floor=bool(forward_speed[parent]<self.minimum-.1)))
        ancestors={child:parent for parent,child in EDGES}
        priority_candidates=set(recovering)
        for member in recovering:
            while member in ancestors:
                member=ancestors[member];priority_candidates.add(member)
        priorities=sorted(priority_candidates-offenders)
        for parent in priorities:
            caps[parent]=self.nominal
            reasons[parent].append(dict(kind='restore_predecessor_order'))
        for e in order:
            parent,child=e['parent'],e['child']
            parent_speed=min(max(self.minimum,float(forward_speed[parent])),float(caps[parent]))
            closing=max(0.,float(forward_speed[child])-parent_speed)
            projected=e['gap_m']-closing*self.lookahead
            e.update(projected_gap_m=projected,predictive_risk=bool(projected<e['desired_gap_m']))
            if projected<e['desired_gap_m']:
                cap=max(self.minimum,parent_speed+(e['gap_m']-e['desired_gap_m'])/self.lookahead)
                if projected<0:cap=self.minimum
                caps[child]=min(caps[child],cap)
                reasons[child].append(dict(kind='predicted_overtake',member=parent,projected_gap_m=projected))
        caps=np.clip(caps,self.minimum,self.nominal)
        return dict(graph_cost=cost,coefficient_kind='squared_normalized_laplacian_xy',
            row_contributions=rows.tolist(),repair_improvements=repairs.tolist(),culprit_members=culprits,
            graph_growth_rates=growth.tolist(),speed_caps=caps.tolist(),speed_reasons=reasons,
            order_edges=order,order_recovery_members=priorities,axis_xy=self.axis.tolist(),order_course_xy=self.order_course.tolist(),
            leader_center=p[2].tolist(),diagnostic_reference_center=center.tolist(),
            desired_offsets_xy=offsets.tolist(),minimum_speed_mps=self.minimum,nominal_speed_mps=self.nominal)
