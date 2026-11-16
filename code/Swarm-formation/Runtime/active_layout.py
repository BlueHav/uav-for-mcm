"""Physical aircraft selection in a fixed ten-aircraft scene."""
import argparse
import json

class ActiveLayout:
    scene_count = 10
    MODES = {'single': ((2,),), 'pair': ((2,), (7,)),
             'five': (tuple(range(5)),), 'ten': (tuple(range(5)), tuple(range(5,10)))}

    def __init__(self, mode='ten'):
        if not isinstance(mode,str) or mode not in self.MODES:
            raise ValueError('execution.mode must be single, pair, five or ten')
        self.mode=mode
        self.groups=self.MODES[mode]
        self.group_ids=tuple(m[0]//5 for m in self.groups)
        self.active_ids=tuple(i for members in self.groups for i in members)
        self.inactive_ids=tuple(i for i in range(self.scene_count) if i not in self.active_ids)
        self.formation=mode in ('five','ten')

    def members(self,group):
        if group not in self.group_ids:raise ValueError('Inactive group')
        return self.groups[self.group_ids.index(group)]

    def complete(self,states,predicate):
        return all(i in states and predicate(states[i]) for i in self.active_ids)

    def as_dict(self):
        return dict(mode=self.mode,scene_count=self.scene_count,active_ids=list(self.active_ids),
                    inactive_ids=list(self.inactive_ids),formation=self.formation,
                    groups=[dict(id=g,leader_id=2+5*g,members=list(self.members(g))) for g in self.group_ids])

    SUPPORTED=('single','pair','five','ten')

    def require_runtime(self):
        # Five is integrated: point_mission_node scopes commands, landing
        # evidence and cruise scoring to active_ids, and the parked bodies no
        # longer veto the cross-group distance band.
        if self.mode not in self.SUPPORTED:
            raise ValueError('Active mode %s runtime is not integrated'%self.mode)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=tuple(ActiveLayout.MODES),default='ten')
    parser.add_argument('--require-runtime',action='store_true')
    args=parser.parse_args();layout=ActiveLayout(args.mode)
    if args.require_runtime:
        try:layout.require_runtime()
        except ValueError as exc:parser.error(str(exc))
    print(json.dumps(layout.as_dict()))
