using System;
using System.Collections.Generic;
using UnityEngine;
namespace Yopo.TrackerTest
{
    public sealed class WorldNavigation
    {
        readonly NavigationData n;
        public WorldNavigation(NavigationData data){n=data;}
        int Cell(Vector3 p) => Mathf.Clamp((int)((p.y+250)/n.cell),0,n.height-1)*n.width+Mathf.Clamp((int)(p.x/n.cell),0,n.width-1);
        Vector3 Point(int k)=>new Vector3((k%n.width+.5f)*n.cell,-250+(k/n.width+.5f)*n.cell,0);
        public List<Vector3> Route(Vector3 start,Vector3 goal)
        {
            if(goal.x<0||goal.x>n.width*n.cell||goal.y< -250||goal.y>250)return new List<Vector3>();
            int a=Cell(start),b=Cell(goal),count=n.width*n.height;
            if(n.blocked[a]=='1'||n.blocked[b]=='1')return new List<Vector3>();
            var costs=new int[count];var prev=new int[count];Array.Fill(costs,int.MaxValue);
            var open=new SortedSet<(int f,int id)>();costs[a]=0;open.Add((0,a));
            int[] delta={1,-1,n.width,-n.width};
            while(open.Count>0)
            {
                var item=open.Min;open.Remove(item);int cur=item.id;
                if(cur==b)
                {
                    var route=new List<Vector3>();
                    for(int k=b;k!=a;k=prev[k])route.Add(Point(k));
                    route.Add(Point(a));route.Reverse();route.Add(new Vector3(goal.x,goal.y,0));return route;
                }
                foreach(int d in delta)
                {
                    int k=cur+d;if(k<0||k>=count||Mathf.Abs(k%n.width-cur%n.width)+Mathf.Abs(k/n.width-cur/n.width)!=1||n.blocked[k]=='1')continue;
                    int nc=costs[cur]+1;if(nc>=costs[k])continue;
                    costs[k]=nc;prev[k]=cur;
                    open.Add((nc+Mathf.Abs(k%n.width-b%n.width)+Mathf.Abs(k/n.width-b/n.width),k));
                }
            }
            return new List<Vector3>();
        }
    }
}

