"""EXPERIMENTAL configured-scale validation; prior geometry passes do NOT apply.

Source ROS and the workspace, then run in Ubuntu Terminal:
    python3 validate_v2_large.py recovery
    python3 validate_v2_large.py reroute
    python3 validate_v2_large.py charging_gap
    python3 validate_v2_large.py priority
    python3 validate_v2_large.py equal
    python3 validate_v2_large.py later
    python3 validate_v2_large.py fleet

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
import sys
import tempfile
import time
import urllib.request

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool
from edge_ai_nav.fleet.route_graph import WarehouseGraph


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('scenario',choices=['recovery','fleet','reroute','charging_gap','motion',
                                            'priority','equal','later','turn','goal_correction'])
    parser.add_argument('--speed',type=float,choices=(0.6,0.9,1.2,1.5,1.8),default=0.6)
    parser.add_argument('--report',default='SWARMX_LARGE_LATEST_GATE.json')
    parser.add_argument('--visible',action='store_true',
                        help='Show Gazebo for recording the same supervised scenario')
    args=parser.parse_args()
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1',8091))==0:
            raise SystemExit('Stop the existing V2 stack first.')
    if subprocess.run(['pgrep','-af','[g]z sim'],capture_output=True).returncode==0:
        raise SystemExit('Stop the existing Gazebo stack first.')
    workspace=Path(__file__).resolve().parent
    config=yaml.safe_load((workspace/'src/amr_simulation/config/warehouse_sih_v2_large.yaml').read_text())['warehouse']
    graph=WarehouseGraph(config)
    static_boxes=config['route_graph']['obstacles']
    poses={}; seen={}; statuses={}; events=[]; completed=set(); shared_decisions={}
    physical_speed={}; world_velocity={}; conflict_metrics={}
    lateral_speed={}; tilt={}; angular_speed={}; hold_cycles={}; align_cycles={}; max_tilt=0.; max_angular=0.; max_lateral=0.
    rclpy.init()
    node=rclpy.create_node('v2_supervised_validation')

    def odom(rid,msg):
        poses[rid]=(msg.pose.pose.position.x,msg.pose.pose.position.y)
        physical_speed[rid]=math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y)
        q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        vx,vy=msg.twist.twist.linear.x,msg.twist.twist.linear.y
        world_velocity[rid]=(vx*math.cos(yaw)-vy*math.sin(yaw),
                             vx*math.sin(yaw)+vy*math.cos(yaw))
        lateral_speed[rid]=abs(msg.twist.twist.linear.y)
        seen[rid]=time.monotonic()

    def imu(rid,msg):
        q=msg.orientation
        roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
        pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
        tilt[rid]=max(abs(roll),abs(pitch))
        angular_speed[rid]=abs(msg.angular_velocity.z)

    def status(rid,msg):
        value=json.loads(msg.data);previous=statuses.get(rid,{})
        # A coordination hold can temporarily replace the published ALIGN
        # state while the controller's hysteretic alignment remains latched.
        # Count only real controller alignment entries, not hold/resume state
        # transitions back into the same episode.
        entered=max(0,int(value.get('align_episodes',0))-
                      int(previous.get('align_episodes',0)))
        if entered:
            key=f"{rid}:{value.get('route_version')}:{value.get('waypoint')}"
            align_cycles[key]=align_cycles.get(key,0)+entered
            if align_cycles[key]>3:
                raise RuntimeError('Persistent corner chasing: '+key)
        if (previous.get('state')=='COORDINATION_HOLD' and value.get('state')=='WAYPOINT_TRACK'
                and value.get('coordination_state') in ('YIELD','YIELD_AND_WAIT','WAIT_FOR_CLEAR')):
            hold_cycles[rid]=hold_cycles.get(rid,0)+1
            if hold_cycles[rid]>1:
                raise RuntimeError('Persistent yield stop/start oscillation: '+rid)
        statuses[rid]=value
        if int(value.get('align_episodes',0))>2*len(value.get('route') or [])+2:
            raise RuntimeError('Persistent waypoint alignment/dancing: '+rid)

    def event(msg):
        value=json.loads(msg.data)
        if value.get('event')=='TASK_COMPLETED':
            completed.add(value['robot_id'])
        if value.get('event') in ('TASK_COMPLETED','YIELD_POCKET_SELECTED','REROUTE_STARTED',
                'YIELD_POCKET_CLEARED','NEGOTIATION_COMPLETE','CONFLICT_RESOLVED'):
            events.append(value)

    def conflict_status(rid,msg):
        data=json.loads(msg.data)
        for conflict in data.get('conflicts',[]):
            cid=conflict['conflict_id']
            peer=conflict['peer'].lower().replace(' ','_')
            if cid not in conflict_metrics and rid in poses and peer in poses:
                distance=math.dist(poses[rid],poses[peer])
                rv=tuple(world_velocity.get(peer,(0.,0.))[i]-world_velocity.get(rid,(0.,0.))[i] for i in (0,1))
                braking=sum(physical_speed.get(r,0.)**2/1.3 for r in (rid,peer))
                closing=max(0.,-sum((poses[peer][i]-poses[rid][i])*rv[i] for i in (0,1))/max(distance,1e-9))
                conflict_metrics[cid]=dict(initial_detection_distance_m=distance,
                    relative_velocity_mps=math.hypot(*rv),closing_velocity_mps=closing,
                    estimated_combined_braking_distance_m=braking,
                    stopping_margin_m=distance-config['stage6']['physical_stop_distance_m']-braking-.35*closing,
                    winner_speed_samples_mps=[],winner_hold_samples=0)
        for decision in data.get('negotiations',[]):
            cid=decision.get('conflict_id')
            if cid:
                shared_decisions.setdefault(cid,{}).setdefault(rid,set()).add((
                    decision.get('winner'),decision.get('loser'),
                    decision.get('decision_version')))
                if cid in conflict_metrics:
                    metric=conflict_metrics[cid]
                    metric.update(winner=decision.get('winner'),loser=decision.get('loser'),
                        loser_action=decision.get('loser_action'),
                        decision_latency_ms=decision.get('decision_latency_ms'))
                    winner=decision.get('winner','').lower().replace(' ','_')
                    if statuses.get(winner,{}).get('state') not in ('DEMO_STAGED','CHARGING'):
                        metric['winner_speed_samples_mps'].append(physical_speed.get(winner,0.))
                        if statuses.get(winner,{}).get('state')=='COORDINATION_HOLD':
                            metric['winner_hold_samples']+=1

    for i in range(1,6):
        rid=f'alpha_{i}'
        node.create_subscription(Odometry,f'/{rid}/odom',
            lambda msg,r=rid:odom(r,msg),qos_profile_sensor_data)
        node.create_subscription(Imu,f'/{rid}/imu',
            lambda msg,r=rid:imu(r,msg),qos_profile_sensor_data)
        node.create_subscription(String,f'/{rid}/local_status',
            lambda msg,r=rid:status(r,msg),10)
        node.create_subscription(String,f'/{rid}/conflict_status',
            lambda msg,r=rid:conflict_status(r,msg),10)
    node.create_subscription(String,'/fleet/coordination_events',event,50)
    publisher=node.create_publisher(String,'/fleet/task_assignment',10)
    log=tempfile.NamedTemporaryFile(prefix='swarmx-v2-'+args.scenario+'-',suffix='.log',delete=False)
    result={'status':'FAIL','scenario':args.scenario,'requested_speed_mps':args.speed,
            'linear_scale':config['scale_speed_upgrade']['linear_scale'],
            'target_speed_mps':config['scale_speed_upgrade']['target_speed_mps'],
            'log':log.name}
    process=None; minimum=float('inf'); min_static=float('inf'); pair_minima={}
    peak_speed=0.; peak_speeds={}; moving_speed_sum=0.; moving_speed_samples=0
    initialized=False; ready_at=None
    brake_client=node.create_client(SetBool,'/alpha_3/autonomy_enabled')
    brake_phase='PENDING';brake_origin=None;brake_start=0.;brake_speed=0.;brake_distance=None
    progress_at=0.; task_index=0; next_task_at=0.; pocket_seen=False
    max_outstanding_assignments=0
    # Dispatch the longest centre-to-left transfer first so its delivery lane
    # clears before the shorter missions begin their automatic charging return.
    # This is the same accepted five-task set, only ordered for fleet liveness.
    pairs=[('PICKUP_10','DROP_03'),('PICKUP_02','DROP_02'),
           ('PICKUP_04','DROP_06'),('PICKUP_01','DROP_01'),('PICKUP_03','DROP_07')]
    # Real controller mission in the free central corridor: straight, 30°,
    # 45° and 90° changes followed by successive opposing corrections.
    motion_route=[[25.,4.],[25.,18.]]
    for degrees,length in ((60,2),(105,2),(165,2),(75,2),(135,2),(45,2),(90,4)):
        angle=math.radians(degrees); x,y=motion_route[-1]
        motion_route.append([x+length*math.cos(angle),y+length*math.sin(angle)])
    if args.scenario=='turn':motion_route=[[25.,16.5],[25.,19.],[27.,19.]]
    if args.scenario=='goal_correction':motion_route=[[25.,16.5],[25.,17.]]
    if args.scenario in ('motion','turn','goal_correction') and not all(graph.visible(a,b)
            for a,b in zip(motion_route,motion_route[1:])):
        raise SystemExit('Motion regression route is not footprint-safe')
    try:
        process=subprocess.Popen([str(workspace/'run_swarmx_v2_large.sh'),
            f'headless:={str(not args.visible).lower()}','enabled:=false','use_rviz:=false',
            f'max_linear_speed:={args.speed}',
            'max_angular_speed:=0.50']+
            (['allow_reroute:=true'] if args.scenario=='reroute' else []),cwd=workspace,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+(120. if args.scenario=='goal_correction' else
                                    240. if args.scenario in ('recovery','charging_gap') else
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
                             [('alpha_3',25.,4. if args.scenario=='motion' else 16.5,
                               0. if args.scenario=='goal_correction' else math.pi/2)]
                             if args.scenario in ('motion','turn','goal_correction') else
                             [('alpha_3',37.775,16.5,-math.pi/2),
                              ('alpha_5',37.775,19.8,-math.pi/2)]
                             if args.scenario=='recovery' else
                             [('alpha_3',42.775,24.4,-math.pi/2),
                              ('alpha_5',37.775,29.,-math.pi/2)]
                             if args.scenario=='reroute' else
                             [('alpha_1',19.,23.5,0.),('alpha_2',25.,32.5,-math.pi/2)])
                    for rid,x,y,yaw in staging:
                        request=(f'name: "{rid}", position: {{x: {x}, y: {y}, z: 0.02}}, '
                                 f'orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}')
                        moved=subprocess.run(['gz','service','-s','/world/warehouse_sih_v2_large/set_pose',
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
            if any(value>.10 for value in tilt.values()):
                raise RuntimeError('Physical tilt stability gate failed: '+str(tilt))
            if any(value>.75 for value in angular_speed.values()):
                raise RuntimeError('Physical angular speed gate failed: '+str(angular_speed))
            max_tilt=max(max_tilt,*tilt.values()) if tilt else max_tilt
            max_angular=max(max_angular,*angular_speed.values()) if angular_speed else max_angular
            max_lateral=max(max_lateral,*lateral_speed.values()) if lateral_speed else max_lateral
            if any(value>.20 for value in lateral_speed.values()):
                raise RuntimeError('Excessive physical side-slip: '+str(lateral_speed))
            pair=min((math.dist(poses[a],poses[b]),a,b)
                     for a,b in itertools.combinations(sorted(poses),2))
            for a,b in itertools.combinations(sorted(poses),2):
                key=a+':'+b
                pair_minima[key]=min(pair_minima.get(key,float('inf')),math.dist(poses[a],poses[b]))
            minimum=min(minimum,pair[0])
            peak_speed=max(peak_speed,*physical_speed.values())
            for rid,speed in physical_speed.items():
                peak_speeds[rid]=max(peak_speeds.get(rid,0.),speed)
            moving=[speed for speed in physical_speed.values() if speed>.1]
            moving_speed_sum+=sum(moving)
            moving_speed_samples+=len(moving)
            static=min(math.hypot(max(box[0]-x,0.,x-box[1]),
                                  max(box[2]-y,0.,y-box[3]))
                       for x,y in poses.values() for box in static_boxes)
            wall_gap=min(min(x-graph.bounds['x'][0],graph.bounds['x'][1]-x,
                             y-graph.bounds['y'][0],graph.bounds['y'][1]-y)
                         for x,y in poses.values())
            static=min(static,wall_gap)
            min_static=min(min_static,static)
            if pair[0]<config['stage6']['physical_stop_distance_m']:
                raise RuntimeError(f'Preemptive separation gate failed: {pair}')
            if static<graph.clearance:
                raise RuntimeError(f'Static obstacle clearance gate failed: {static:.3f} m')
            if args.scenario=='charging_gap' and task_index and poses['alpha_2'][0]>24.0 and poses['alpha_2'][1]>33.2:
                raise RuntimeError('Dock-entry corner overshot into occupied bay approach: '+
                                   str((poses['alpha_2'],statuses['alpha_2'].get('linear'),
                                        statuses['alpha_2'].get('angular'),
                                        statuses['alpha_2'].get('lookahead_target'))))
            if args.scenario=='recovery' and task_index==0:
                for rid,target,priority in [('alpha_3',[45.,16.5],2),('alpha_5',[37.775,8.],4)]:
                    publisher.publish(String(data=json.dumps(dict(robot_id=rid,
                        task='RECOVERY_VALIDATION',task_id='RECOVER-'+rid,
                        route=[list(poses[rid]),target],priority=priority,demo_hold=True))))
                task_index=2
            if args.scenario in ('motion','turn','goal_correction') and task_index==0:
                # set_pose can settle a few centimetres before the task is
                # published.  Start from measured odometry so waypoint zero
                # cannot remain behind the robot and masquerade as corner
                # chasing at the highest speed gate.
                measured_route=[list(poses['alpha_3']),*motion_route[1:]]
                publisher.publish(String(data=json.dumps(dict(robot_id='alpha_3',
                    task='MOTION_VALIDATION',task_id='MOTION-alpha_3',
                    route=measured_route,priority=3,demo_hold=True))))
                task_index=1
            if args.scenario=='motion' and task_index:
                if (brake_phase=='PENDING' and physical_speed.get('alpha_3',0.)>=args.speed*.99
                        and statuses['alpha_3'].get('waypoint')==1 and brake_client.service_is_ready()):
                    brake_phase='STOPPING';brake_origin=poses['alpha_3']
                    brake_speed=physical_speed['alpha_3'];brake_start=now
                    request=SetBool.Request();request.data=False
                    brake_client.call_async(request)
                elif (brake_phase=='STOPPING' and now-brake_start>.5
                      and physical_speed.get('alpha_3',1.)<.03):
                    brake_distance=math.dist(brake_origin,poses['alpha_3'])
                    allowed=brake_speed*brake_speed/(2*.65)+.35*brake_speed+.10
                    if brake_distance>allowed:
                        raise RuntimeError(f'Measured physical braking exceeded estimate: {brake_distance} > {allowed}')
                    brake_phase='COMPLETE'
                    request=SetBool.Request();request.data=True
                    brake_client.call_async(request)
            if args.scenario=='charging_gap' and task_index==0:
                publisher.publish(String(data=json.dumps(dict(robot_id='alpha_2',
                    task='ADJACENT_BAY_VALIDATION',task_id='CHARGE-GAP-alpha_2',
                    # Exercise the exact north-cross-aisle right-angle corner
                    # that stalled the full fleet next to occupied CHARGE_03.
                    route=[list(poses['alpha_2']),[25.,32.5],[20.2,32.5],[20.2,37.4]],
                    priority=3,
                    destination='CHARGE_02',demo_hold=True))))
                task_index=1
            if args.scenario=='reroute' and task_index==0:
                missions=[('alpha_5',[[37.775,29.],[37.775,8.]],'RIGHT_INNER_LOWER',4),
                          ('alpha_3',[[42.775,24.4],[42.775,23.5],[37.775,23.5],
                                      [37.775,8.],[45.,4.]],'DROP_07',2)]
                for rid,route,destination,priority in missions:
                    publisher.publish(String(data=json.dumps(dict(robot_id=rid,
                        task='REROUTE_VALIDATION',task_id='REROUTE-'+rid,route=route,
                        priority=priority,destination=destination,demo_hold=True))))
                task_index=2
            if args.scenario in ('priority','equal','later') and task_index in (0,4):
                stage_y=16.5 if task_index==4 else 23.5
                for rid,route,priority in (
                    ('alpha_1',[[19.,stage_y],[25.,stage_y],[31.,stage_y]],4 if args.scenario!='equal' else 3),
                    ('alpha_2',[[25.,32.5],[25.,stage_y],[25.,14.5]],2 if args.scenario!='equal' else 3)):
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
                task_index+=1
                # The 2x bodies need enough time to leave the five adjacent
                # charging bays before the next AUTO auction.  These long
                # warehouse transfers still overlap at a 40 s interval (the
                # validator requires five outstanding at once), without
                # manufacturing a four-way queue at the shared north aisle.
                next_task_at=now+40.
            if args.scenario=='fleet':
                max_outstanding_assignments=max(
                    max_outstanding_assignments,task_index-len(completed))
            pocket_seen=pocket_seen or any(s['state']=='YIELD_POCKET' for s in statuses.values())
            if args.scenario in ('turn','goal_correction'):
                done=statuses['alpha_3']['state']=='DEMO_STAGED'
            elif args.scenario=='motion':
                done=statuses['alpha_3']['state']=='DEMO_STAGED' and brake_phase=='COMPLETE'
            elif args.scenario=='recovery':
                done=(pocket_seen and all(statuses[r]['state']=='DEMO_STAGED'
                                         for r in ('alpha_3','alpha_5')))
            elif args.scenario=='charging_gap':
                done=(statuses['alpha_2']['state']=='DEMO_STAGED' and
                      statuses['alpha_3']['state']=='CHARGING')
            elif args.scenario=='reroute':
                reroutes=[e for e in events if e['event']=='REROUTE_STARTED']
                route=statuses['alpha_3'].get('route') or []
                bypass=([47.775,23.5] in route and [47.775,16.5] in route and
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
                        moved=subprocess.run(['gz','service','-s','/world/warehouse_sih_v2_large/set_pose',
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
                done=(len(completed)==5 and max_outstanding_assignments==5 and
                      all(s['state']=='CHARGING' for s in statuses.values()))
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
                try: process.wait(timeout=10.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait(timeout=5.)
        if process is not None:
            # Gazebo's GUI helper may outlive its ROS launch parent. This is
            # the isolated process group created by this validator, never a
            # broad kill of unrelated ROS/Gazebo processes.
            try:
                os.killpg(process.pid,signal.SIGTERM)
                time.sleep(.5)
                os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:
                pass
        log.close()
    for metric in conflict_metrics.values():
        samples=metric.pop('winner_speed_samples_mps')
        metric.update(winner_speed_sample_count=len(samples),
            winner_peak_speed_mps=max(samples) if samples else None,
            winner_mean_sampled_speed_mps=sum(samples)/len(samples) if samples else None)
    result.update(latest_tilt_rad=tilt, latest_angular_speed=angular_speed,
        braking_phase=brake_phase,braking_start_speed_mps=brake_speed,
        measured_stopping_distance_m=brake_distance,
        maximum_tilt_rad=max_tilt,maximum_angular_speed_rad_s=max_angular,yield_hold_cycles=hold_cycles,
        maximum_lateral_speed_mps=max_lateral,
        corner_align_cycles=align_cycles,
        minimum_sampled_separation_m=round(minimum,3) if math.isfinite(minimum) else None,
        minimum_sampled_pair_separations_m=pair_minima,
        minimum_sampled_static_gap_m=round(min_static,3) if math.isfinite(min_static) else None,
        peak_physical_speed_mps=round(peak_speed,3),pocket_seen=pocket_seen,
        peak_physical_speeds_mps=peak_speeds,
        mean_moving_physical_speed_mps=round(
            moving_speed_sum/max(1,moving_speed_samples),3),
        completed=sorted(completed),events=events,
        maximum_concurrent_outstanding_tasks=max_outstanding_assignments,
        conflict_metrics=conflict_metrics,
        shared_decisions={cid:{rid:[list(value) for value in values]
                               for rid,values in sides.items()}
                          for cid,sides in shared_decisions.items()},
        final={r:dict(pose=poses.get(r),state=s['state'],waypoint=s['waypoint'],
                     route=s.get('route')) for r,s in statuses.items()})
    report=workspace/args.report
    result['validator_cleanup']='Observer process exits after stopping its owned simulation'
    report.write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items()
                      if k not in ('events','final','shared_decisions','conflict_metrics')}),flush=True)
    print('Evidence: '+str(report),flush=True)
    # The observer's native ROS destructor can hold Python's GIL indefinitely,
    # making even a thread timeout ineffective. All owned Gazebo/ROS processes
    # are already stopped. Export first, then exit this disposable observer
    # process only; the OS releases its DDS handles. No prototype DDS changes.
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(0 if result['status']=='PASS' else 1)


if __name__=='__main__':
    raise SystemExit(main())
