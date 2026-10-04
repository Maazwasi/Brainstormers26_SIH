"""Supervised physical V2 checks, using live ROS odometry and local decisions.

Source ROS and the workspace, then run in Ubuntu Terminal:
    python3 validate_v2_fleet.py recovery
    python3 validate_v2_fleet.py reroute
    python3 validate_v2_fleet.py charging_gap
    python3 validate_v2_fleet.py priority
    python3 validate_v2_fleet.py equal
    python3 validate_v2_fleet.py later
    python3 validate_v2_fleet.py fleet

Recovery stages poses only before assigning tasks. Fleet uses normal AUTO
dashboard tasks from the default spawns. Every exit stops the owned stack.
"""
import argparse
import itertools
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import urllib.request

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from edge_ai_nav.fleet.route_graph import WarehouseGraph


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('scenario',choices=['recovery','fleet','reroute','charging_gap','motion',
                                            'priority','equal','later'])
    parser.add_argument('--speed',type=float,choices=(0.4,0.5,0.6),default=0.4)
    parser.add_argument('--visible',action='store_true',
                        help='Show Gazebo for recording the same supervised scenario')
    args=parser.parse_args()
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1',8091))==0:
            raise SystemExit('Stop the existing V2 stack first.')
    if subprocess.run(['pgrep','-af','[g]z sim'],capture_output=True).returncode==0:
        raise SystemExit('Stop the existing Gazebo stack first.')
    workspace=Path(__file__).resolve().parent
    config=yaml.safe_load((workspace/'src/amr_simulation/config/warehouse_sih_v2.yaml').read_text())['warehouse']
    graph=WarehouseGraph(config)
    static_boxes=config['route_graph']['obstacles']
    poses={}; seen={}; statuses={}; events=[]; completed=set(); shared_decisions={}
    physical_speed={}
    rclpy.init()
    node=rclpy.create_node('v2_supervised_validation')

    def odom(rid,msg):
        poses[rid]=(msg.pose.pose.position.x,msg.pose.pose.position.y)
        physical_speed[rid]=math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y)
        seen[rid]=time.monotonic()

    def status(rid,msg):
        statuses[rid]=json.loads(msg.data)

    def event(msg):
        value=json.loads(msg.data)
        if value.get('event')=='TASK_COMPLETED':
            completed.add(value['robot_id'])
        if value.get('event') in ('TASK_COMPLETED','YIELD_POCKET_SELECTED','REROUTE_STARTED',
                'YIELD_POCKET_CLEARED','NEGOTIATION_COMPLETE','CONFLICT_RESOLVED'):
            events.append(value)

    def conflict_status(rid,msg):
        for decision in json.loads(msg.data).get('negotiations',[]):
            cid=decision.get('conflict_id')
            if cid:
                shared_decisions.setdefault(cid,{}).setdefault(rid,set()).add((
                    decision.get('winner'),decision.get('loser'),
                    decision.get('decision_version')))

    for i in range(1,6):
        rid=f'alpha_{i}'
        node.create_subscription(Odometry,f'/{rid}/odom',
            lambda msg,r=rid:odom(r,msg),qos_profile_sensor_data)
        node.create_subscription(String,f'/{rid}/local_status',
            lambda msg,r=rid:status(r,msg),10)
        node.create_subscription(String,f'/{rid}/conflict_status',
            lambda msg,r=rid:conflict_status(r,msg),10)
    node.create_subscription(String,'/fleet/coordination_events',event,50)
    publisher=node.create_publisher(String,'/fleet/task_assignment',10)
    log=tempfile.NamedTemporaryFile(prefix='swarmx-v2-'+args.scenario+'-',suffix='.log',delete=False)
    result={'status':'FAIL','scenario':args.scenario,'requested_speed_mps':args.speed,
            'log':log.name}
    process=None; minimum=float('inf'); min_static=float('inf')
    peak_speed=0.; moving_speed_sum=0.; moving_speed_samples=0
    initialized=False; ready_at=None
    progress_at=0.; task_index=0; next_task_at=0.; pocket_seen=False
    pairs=[('PICKUP_03','DROP_07'),('PICKUP_01','DROP_01'),
           ('PICKUP_04','DROP_06'),('PICKUP_02','DROP_02'),('PICKUP_10','DROP_03')]
    # Real controller mission in the free central corridor: straight, 30°,
    # 45° and 90° changes followed by successive opposing corrections.
    motion_route=[[21.,10.]]
    for degrees,length in ((0,4),(30,4),(75,3),(165,3),(195,3),
                           (105,3),(15,3),(30,3)):
        angle=math.radians(degrees); x,y=motion_route[-1]
        motion_route.append([x+length*math.cos(angle),y+length*math.sin(angle)])
    if args.scenario=='motion' and not all(graph.visible(a,b)
            for a,b in zip(motion_route,motion_route[1:])):
        raise SystemExit('Motion regression route is not footprint-safe')
    try:
        process=subprocess.Popen([str(workspace/'run_swarmx_v2_demo.sh'),
            f'headless:={str(not args.visible).lower()}','enabled:=false','use_rviz:=false',
            f'max_linear_speed:={args.speed}',
            f'max_angular_speed:={0.75 if args.speed==0.4 else 0.85 if args.speed==0.5 else 0.9}']+
            (['allow_reroute:=true'] if args.scenario=='reroute' else []),cwd=workspace,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+(240. if args.scenario in ('recovery','charging_gap') else
                                    420. if args.scenario in ('priority','equal','later') else
                                    1800. if args.scenario=='fleet' else 1100.)
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            now=time.monotonic()
            if process.poll() is not None:
                raise RuntimeError('Simulation process exited')
            if len(poses)<5 or len(statuses)<5 or publisher.get_subscription_count()<5:
                continue
            if not initialized:
                ready_at=now+2.
                if args.scenario != 'fleet':
                    staging=([('alpha_2',25.,27.,math.pi/2)]
                             if args.scenario=='charging_gap' else
                             [('alpha_3',21.,10.,0.)] if args.scenario=='motion' else
                             [('alpha_3',38.25,16.5,-math.pi/2),
                              ('alpha_5',38.25,19.8,-math.pi/2)]
                             if args.scenario=='recovery' else
                             [('alpha_3',41.,25.1,-math.pi/2),
                              ('alpha_5',38.25,29.,-math.pi/2)]
                             if args.scenario=='reroute' else
                             [('alpha_1',20.,23.5,0.),('alpha_2',30.,23.5,math.pi)])
                    for rid,x,y,yaw in staging:
                        request=(f'name: "{rid}", position: {{x: {x}, y: {y}, z: 0.02}}, '
                                 f'orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}')
                        moved=subprocess.run(['gz','service','-s','/world/warehouse_sih_v2/set_pose',
                            '--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean',
                            '--timeout','3000','--req',request],capture_output=True,text=True,timeout=5.)
                        if moved.returncode or 'true' not in moved.stdout:
                            raise RuntimeError('Could not stage recovery poses: '+moved.stdout)
                initialized=True
                continue
            if now<ready_at:
                continue
            if now-min(seen.values())>2.:
                raise RuntimeError('Stale physical odometry')
            pair=min((math.dist(poses[a],poses[b]),a,b)
                     for a,b in itertools.combinations(sorted(poses),2))
            minimum=min(minimum,pair[0])
            peak_speed=max(peak_speed,*physical_speed.values())
            moving=[speed for speed in physical_speed.values() if speed>.1]
            moving_speed_sum+=sum(moving)
            moving_speed_samples+=len(moving)
            static=min(math.hypot(max(box[0]-x,0.,x-box[1]),
                                  max(box[2]-y,0.,y-box[3]))
                       for x,y in poses.values() for box in static_boxes)
            min_static=min(min_static,static)
            if pair[0]<1.5:
                raise RuntimeError(f'Preemptive separation gate failed: {pair}')
            if static<graph.clearance:
                raise RuntimeError(f'Static obstacle clearance gate failed: {static:.3f} m')
            if args.scenario=='charging_gap' and task_index and poses['alpha_2'][0]>24.0 and poses['alpha_2'][1]>33.2:
                raise RuntimeError('Dock-entry corner overshot into occupied bay approach: '+
                                   str((poses['alpha_2'],statuses['alpha_2'].get('linear'),
                                        statuses['alpha_2'].get('angular'),
                                        statuses['alpha_2'].get('lookahead_target'))))
            if args.scenario=='recovery' and task_index==0:
                for rid,target,priority in [('alpha_3',[45.,16.5],2),('alpha_5',[38.25,8.],4)]:
                    publisher.publish(String(data=json.dumps(dict(robot_id=rid,
                        task='RECOVERY_VALIDATION',task_id='RECOVER-'+rid,
                        route=[list(poses[rid]),target],priority=priority,demo_hold=True))))
                task_index=2
            if args.scenario=='motion' and task_index==0:
                publisher.publish(String(data=json.dumps(dict(robot_id='alpha_3',
                    task='MOTION_VALIDATION',task_id='MOTION-alpha_3',
                    route=motion_route,priority=3,demo_hold=True))))
                task_index=1
            if args.scenario=='charging_gap' and task_index==0:
                publisher.publish(String(data=json.dumps(dict(robot_id='alpha_2',
                    task='ADJACENT_BAY_VALIDATION',task_id='CHARGE-GAP-alpha_2',
                    # Exercise the exact north-cross-aisle right-angle corner
                    # that stalled the full fleet next to occupied CHARGE_03.
                    route=[list(poses['alpha_2']),[25.,32.5],[22.,32.5],[22.,36.]],
                    priority=3,
                    destination='CHARGE_02',demo_hold=True))))
                task_index=1
            if args.scenario=='reroute' and task_index==0:
                missions=[('alpha_5',[[38.25,29.],[38.25,8.]],'RIGHT_INNER_LOWER',4),
                          ('alpha_3',[[41.,25.1],[41.,23.5],[38.25,23.5],
                                      [38.25,8.],[45.,4.]],'DROP_07',2)]
                for rid,route,destination,priority in missions:
                    publisher.publish(String(data=json.dumps(dict(robot_id=rid,
                        task='REROUTE_VALIDATION',task_id='REROUTE-'+rid,route=route,
                        priority=priority,destination=destination,demo_hold=True))))
                task_index=2
            if args.scenario in ('priority','equal','later') and task_index in (0,4):
                stage_y=16.5 if task_index==4 else 23.5
                for rid,route,priority in (
                    ('alpha_1',[[20.,stage_y],[25.,stage_y],[30.,stage_y]],4 if args.scenario!='equal' else 3),
                    ('alpha_2',[[30.,stage_y],[25.,stage_y],[20.,stage_y]],2 if args.scenario!='equal' else 3)):
                    publisher.publish(String(data=json.dumps(dict(robot_id=rid,
                        task='PRIORITY_VALIDATION',
                        task_id=f'{args.scenario}-{task_index}-{rid}',route=route,
                        priority=priority,demo_hold=True))))
                task_index=5 if task_index==4 else 2
            if args.scenario=='fleet' and task_index<5 and now>=next_task_at:
                pickup,destination=pairs[task_index]
                data=json.dumps(dict(type='WAREHOUSE_TRANSFER',pickup=pickup,
                    destination=destination,priority=3,assignment_mode='AUTO')).encode()
                request=urllib.request.Request('http://localhost:8091/api/tasks',data=data,
                    headers={'Content-Type':'application/json'})
                with urllib.request.urlopen(request,timeout=3.) as response:
                    assigned=json.load(response)
                if not assigned.get('success'):
                    raise RuntimeError('Assignment failed: '+str(assigned))
                task_index+=1; next_task_at=now+3.
            pocket_seen=pocket_seen or any(s['state']=='YIELD_POCKET' for s in statuses.values())
            if args.scenario=='motion':
                done=statuses['alpha_3']['state']=='DEMO_STAGED'
            elif args.scenario=='recovery':
                done=(pocket_seen and all(statuses[r]['state']=='DEMO_STAGED'
                                         for r in ('alpha_3','alpha_5')))
            elif args.scenario=='charging_gap':
                done=(statuses['alpha_2']['state']=='DEMO_STAGED' and
                      statuses['alpha_3']['state']=='CHARGING')
            elif args.scenario=='reroute':
                reroutes=[e for e in events if e['event']=='REROUTE_STARTED']
                route=statuses['alpha_3'].get('route') or []
                bypass=([47.5,23.5] in route and [47.5,16.5] in route and
                        route[-1:]==[[45.,4.]] and
                        all(b[1]<=a[1]+.01 for a,b in zip(route,route[1:])))
                done=(len(reroutes)==1 and bypass and
                      all(statuses[r]['state']=='DEMO_STAGED'
                          for r in ('alpha_3','alpha_5')))
            elif args.scenario in ('priority','equal','later'):
                upper=[e for e in events if e['event']=='NEGOTIATION_COMPLETE'
                       and e.get('zone')=='MAIN_UPPER_CROSS'
                       and {e.get('winner'),e.get('loser')}=={'ALPHA 1','ALPHA 2'}]
                expected=('TASK_PRIORITY' if args.scenario!='equal' else None)
                cid='MAIN_UPPER_CROSS:ALPHA 1:ALPHA 2'
                adopted=shared_decisions.get(cid,{})
                agreement=bool(adopted.get('alpha_1',set()) & adopted.get('alpha_2',set()))
                equal_basis=('CHARGING_URGENCY','TASK_DEADLINE','LOADED',
                             'MISSION_PROGRESS','WAITING_TIME','CLEARANCE_TIME','ROBOT_ID')
                first=bool(upper and agreement and
                    (args.scenario=='equal' or upper[0]['winner']=='ALPHA 1') and
                    (expected is None or upper[0]['decision_basis']==expected) and
                    (args.scenario!='equal' or upper[0]['decision_basis'] in equal_basis) and
                    (args.scenario!='equal' or upper[0]['winner_priority']==upper[0]['loser_priority']==3) and
                    upper[0]['loser_action']=='YIELD_AND_WAIT')
                staged=all(statuses[r]['state']=='DEMO_STAGED'
                           for r in ('alpha_1','alpha_2'))
                if args.scenario=='later' and first and staged and task_index==2:
                    # A new encounter uses a different physical cross aisle;
                    # only initial poses are staged, never decisions or states.
                    for rid,x,yaw in (('alpha_1',20.,0.),('alpha_2',30.,math.pi)):
                        request=(f'name: "{rid}", position: {{x: {x}, y: 16.5, z: 0.02}}, '
                                 f'orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}')
                        moved=subprocess.run(['gz','service','-s','/world/warehouse_sih_v2/set_pose',
                            '--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean',
                            '--timeout','3000','--req',request],capture_output=True,text=True,timeout=5.)
                        if moved.returncode or 'true' not in moved.stdout:
                            raise RuntimeError('Could not stage second encounter')
                    task_index=4; ready_at=now+2.
                later=[e for e in events if e['event']=='NEGOTIATION_COMPLETE'
                       and e.get('zone')=='MAIN_MID_CROSS'
                       and {e.get('winner'),e.get('loser')}=={'ALPHA 1','ALPHA 2'}]
                done=(first and staged and
                      (args.scenario!='later' or bool(later and task_index==5)))
            else:
                done=len(completed)==5 and all(s['state']=='CHARGING' for s in statuses.values())
            if done:
                result['status']='PASS'
                break
            if now>=progress_at:
                print(json.dumps(dict(minimum_m=round(minimum,3),closest=pair,
                    completed=len(completed),states={r:(s['state'],s['waypoint'])
                    for r,s in statuses.items()},
                    charging_corner=({k:statuses['alpha_2'].get(k)
                                      for k in ('pose','linear','angular','lookahead_target')}
                                     if args.scenario=='charging_gap' else None))),flush=True)
                progress_at=now+(5. if args.scenario=='charging_gap' else 15.)
        else:
            raise RuntimeError('Validation deadline exceeded')
    except (RuntimeError,OSError,KeyboardInterrupt,subprocess.TimeoutExpired) as error:
        result['reason']=str(error) or 'Interrupted'
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                process.wait(timeout=10.)
        log.close(); node.destroy_node(); rclpy.shutdown()
    result.update(minimum_sampled_separation_m=round(minimum,3),
        minimum_sampled_static_gap_m=round(min_static,3),
        peak_physical_speed_mps=round(peak_speed,3),pocket_seen=pocket_seen,
        mean_moving_physical_speed_mps=round(
            moving_speed_sum/max(1,moving_speed_samples),3),
        completed=sorted(completed),events=events,
        shared_decisions={cid:{rid:[list(value) for value in values]
                               for rid,values in sides.items()}
                          for cid,sides in shared_decisions.items()},
        final={r:dict(pose=poses.get(r),state=s['state'],waypoint=s['waypoint'],
                     route=s.get('route')) for r,s in statuses.items()})
    report=Path(log.name).with_suffix('.json')
    report.write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items()
                      if k not in ('events','final','shared_decisions')}),flush=True)
    print('Evidence: '+str(report),flush=True)
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())
