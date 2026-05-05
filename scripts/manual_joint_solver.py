#!/usr/bin/env python3
from __future__ import annotations
import argparse, re
from collections import deque

DIRS = {'N':(-1,0),'S':(1,0),'E':(0,1),'W':(0,-1)}

MOVE_RE = re.compile(r'^(NoOp|Move\(([NSEW])\)|Push\(([NSEW]),([NSEW])\)|Pull\(([NSEW]),([NSEW])\))$')

def add(p,d):
    return (p[0]+d[0], p[1]+d[1])

def parse_level(path):
    lines=[l.rstrip('\n') for l in open(path)]
    i=lines.index('#colors')+1
    colors={}
    while not lines[i].startswith('#'):
        if lines[i].strip():
            c,vals=lines[i].split(':')
            for v in vals.split(','):
                colors[v.strip()]=c.strip()
        i+=1
    init_i=lines.index('#initial')+1
    goal_i=lines.index('#goal')+1
    end_i=lines.index('#end')
    init=lines[init_i:goal_i-1]
    goal=lines[goal_i:end_i]
    h,w=len(init),len(init[0])
    walls=set(); agents={}; boxes={}; goals={}
    for r in range(h):
        for c,ch in enumerate(init[r]):
            if ch=='+': walls.add((r,c))
            elif ch.isdigit(): agents[int(ch)]=(r,c)
            elif ch.isalpha() and ch.isupper(): boxes[(r,c)]=ch
    for r in range(h):
        for c,ch in enumerate(goal[r]):
            if ch.isalpha() and ch.isupper(): goals[ch]=(r,c)
    n=max(agents)+1
    return walls,agents,boxes,goals,colors,h,w,n

def parse_action(s):
    s=s.strip(); m=MOVE_RE.match(s)
    if not m: raise ValueError(f'Bad action {s}')
    if s=='NoOp': return ('NoOp',None,None)
    if s.startswith('Move'): return ('Move',m.group(2),None)
    if s.startswith('Push'): return ('Push',m.group(3),m.group(4))
    return ('Pull',m.group(5),m.group(6))

def apply_joint(state,joint,walls,h,w):
    agents,boxes=state
    intents=[]
    boxes=dict(boxes)
    for aid,a in enumerate(joint):
        apos=agents[aid]; typ,d1,d2=a
        if typ=='NoOp': intents.append((aid,apos,apos,None,None)); continue
        if typ=='Move':
            to=add(apos,DIRS[d1]); intents.append((aid,apos,to,None,None)); continue
        if typ=='Push':
            bpos=add(apos,DIRS[d1]); bto=add(bpos,DIRS[d2]); intents.append((aid,apos,bpos,bpos,bto)); continue
        if typ=='Pull':
            to=add(apos,DIRS[d1]); bpos=add(apos,DIRS[d2]); intents.append((aid,apos,to,bpos,apos)); continue

    occupied=set(boxes)|set(agents.values())
    # individual legality precheck
    for aid,frm,to,bfrom,bto in intents:
        if to[0]<0 or to[1]<0 or to[0]>=h or to[1]>=w or to in walls: return None
        if bfrom is None:
            if to!=frm and to in occupied: return None
        else:
            if bfrom not in boxes: return None
            if bto[0]<0 or bto[1]<0 or bto[0]>=h or bto[1]>=w or bto in walls: return None
            if bto in occupied and bto!=bfrom: return None

    dests=[i[2] for i in intents]
    if len(set(dests))<len(dests): return None
    # no swap
    for i in range(len(intents)):
        for j in range(i+1,len(intents)):
            if intents[i][1]==intents[j][2] and intents[j][1]==intents[i][2]: return None
    box_from=[i[3] for i in intents if i[3] is not None]
    if len(set(box_from))<len(box_from): return None
    box_to=[i[4] for i in intents if i[4] is not None]
    if len(set(box_to))<len(box_to): return None

    new_agents=dict(agents)
    for aid,frm,to,bfrom,bto in intents: new_agents[aid]=to
    for aid,frm,to,bfrom,bto in intents:
        if bfrom is not None:
            ch=boxes.pop(bfrom)
            boxes[bto]=ch
    return new_agents,boxes

def solved(boxes,goals):
    for ch,g in goals.items():
        if boxes.get(g)!=ch: return False
    return True

def encode(state,n):
    agents,boxes=state
    return tuple(agents[i] for i in range(n)), tuple(sorted(boxes.items()))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('level')
    ap.add_argument('--plan', help='text file with one joint action per line, comma-separated by agent id order')
    ap.add_argument('--search', action='store_true')
    ap.add_argument('--max-depth', type=int, default=60)
    args=ap.parse_args()
    walls,agents,boxes,goals,colors,h,w,n=parse_level(args.level)
    state=(agents,boxes)

    if args.search:
        acts=['NoOp']+[f'Move({d})' for d in 'NSEW']
        acts+=[f'Push({a},{b})' for a in 'NSEW' for b in 'NSEW']
        acts+=[f'Pull({a},{b})' for a in 'NSEW' for b in 'NSEW']
        parsed=[(a,parse_action(a)) for a in acts]
        start=encode(state,n)
        q=deque([(state,[])])
        seen={start}
        while q:
            s,path=q.popleft()
            if solved(s[1],goals):
                print('Solved in',len(path),'steps')
                for i,ja in enumerate(path,1): print(f'{i:02d}: '+', '.join(ja))
                return
            if len(path)>=args.max_depth: continue
            for a0s,a0 in parsed:
                for a1s,a1 in parsed if n==2 else [parsed[0]]:
                    ja=[a0,a1] if n==2 else [a0]
                    ns=apply_joint(s,ja,walls,h,w)
                    if ns is None: continue
                    code=encode(ns,n)
                    if code in seen: continue
                    seen.add(code)
                    q.append((ns,path+[[a0s,a1s]]))
        print('No solution found')
        return

    if not args.plan:
        raise SystemExit('--plan required unless --search')
    plan_lines=[l.strip() for l in open(args.plan) if l.strip() and not l.strip().startswith('#')]
    for t,line in enumerate(plan_lines,1):
        parts=[parse_action(x.strip()) for x in line.split(';')]
        if len(parts)!=n: raise SystemExit(f'Line {t}: expected {n} actions')
        ns=apply_joint(state,parts,walls,h,w)
        if ns is None:
            print(f'Step {t} illegal: {line}')
            return
        state=ns
    print('Final solved:', solved(state[1],goals))

if __name__=='__main__':
    main()
