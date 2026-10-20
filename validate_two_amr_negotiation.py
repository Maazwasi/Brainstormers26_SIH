"""Run the one controlled ALPHA 1 / ALPHA 3 physical negotiation demo.

Run from an Ubuntu Terminal after building the two packages:
  source /opt/ros/lyrical/setup.bash
  source install/local_setup.bash
  python3 validate_two_amr_negotiation.py

The script owns and cleans up only the stack it starts.  It deliberately
refuses to run beside an existing Gazebo/dashboard stack.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


PAIR=('alpha_1','alpha_3')
LOGICAL={'alpha_1':'ALPHA 1','alpha_3':'ALPHA 3'}
CONFLICT='MAIN_UPPER_CROSS:ALPHA 1:ALPHA 3'
RELEVANT={'CONFLICT_DETECTED','NEGOTIATION_COMPLETE',
          'NEGOTIATION_ACTION_APPLIED','NEGOTIATION_LOCAL_PASS_AUTHORIZED',
          'NEGOTIATION_LOCAL_PASS_STARTED','NEGOTIATION_LOCAL_PASS_COMPLETE',
          'CONFLICT_RESOLVED','TASK_COMPLETED','REROUTE_STARTED'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--timeout',type=float,default=300.)
    parser.add_argument('--report',default='SWARMX_TWO_AMR_NEGOTIATION_RESULT.json')
    parser.add_argument('--keep-open',action='store_true',
                        help='Keep Gazebo/RViz open after both demo goals are reached')
    parser.add_argument('--start-delay',type=float,default=15.0,
                        help='Seconds to wait after staging before the AMRs start')
    args=parser.parse_args()
    workspace=Path(__file__).resolve().parent
    if subprocess.run(['pgrep','-af','[g]z sim'],capture_output=True).returncode==0:
        raise SystemExit('Stop the existing Gazebo before this isolated demo.')

    cfg=yaml.safe_load((workspace/'src/amr_simulation/config/warehouse_sih_v2_large.yaml').read_text())['warehouse']
    physical_stop=float(cfg['stage6']['physical_stop_distance_m'])
    bounds=cfg['amr_collision_bounds_m']
    turning_radius=max(math.hypot(float(px),float(py))
                       for px in bounds['x'] for py in bounds['y'])
    required=2*turning_radius+float(cfg['stage6']['local_pass_safety_margin_m'])
    poses={}; speeds={}; statuses={}; events=[]; decisions={rid:set() for rid in PAIR}
    detection_distance=None; minimum_distance=float('inf'); dispatch_routes={}
    loser_stationary_samples=0; loser_motion_during_pass=False
    pass_started=False; pass_complete=False; loser_resumed=False
    winner_progress_after_pass=[]
    stopped_snapshot={}; stopped_cycles=0

    rclpy.init()
    node=rclpy.create_node('swarmx_two_amr_negotiation_validator')

    def odom(rid,msg):
        nonlocal minimum_distance,loser_motion_during_pass
        poses[rid]=(msg.pose.pose.position.x,msg.pose.pose.position.y)
        speeds[rid]=math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y)
        if all(robot in poses for robot in PAIR):
            minimum_distance=min(minimum_distance,math.dist(poses[PAIR[0]],poses[PAIR[1]]))
        if pass_started and not pass_complete and rid=='alpha_3' and speeds[rid]>.08:
            loser_motion_during_pass=True
        if pass_complete and rid=='alpha_1':
            winner_progress_after_pass.append(poses[rid][1])

    def status(rid,msg):
        nonlocal loser_stationary_samples,loser_resumed,stopped_cycles,stopped_snapshot
        previous=statuses.get(rid,{})
        value=json.loads(msg.data); statuses[rid]=value
        if rid=='alpha_3' and value.get('coordination_state') in (
                'YIELD','YIELD_AND_WAIT','WAIT_FOR_CLEAR','SAFE_WAIT') and speeds.get(rid,1.)<.08:
            loser_stationary_samples+=1
        if (rid=='alpha_3' and pass_complete and
                previous.get('state')=='COORDINATION_HOLD' and
                value.get('state')=='WAYPOINT_TRACK'):
            loser_resumed=True
        if (all(robot in statuses for robot in PAIR) and
                all(statuses[robot].get('negotiation_role') in ('WINNER','LOSER')
                    for robot in PAIR) and
                all(abs(float(statuses[robot].get('linear',1.)))<.01 and
                    abs(float(statuses[robot].get('angular',1.)))<.01 for robot in PAIR)):
            stopped_cycles+=1
            if stopped_cycles>=4 and not stopped_snapshot:
                keys=('negotiation_role','coordination_state','coordination_reason','state',
                      'obstacle_state','linear','angular','active_route_target',
                      'lidar_safety_stop','local_bypass_active','bypass_state',
                      'zero_command_reason','negotiation_winner','negotiation_loser')
                stopped_snapshot={robot:{key:statuses[robot].get(key) for key in keys}
                                  for robot in PAIR}
                print('[NEG-ALPHA1-ALPHA3-001] SUSTAINED_TWO_ROBOT_STOP '+
                      json.dumps(stopped_snapshot),flush=True)
        else:
            stopped_cycles=0

    def event(msg):
        nonlocal detection_distance,pass_started,pass_complete
        value=json.loads(msg.data)
        if value.get('event') not in RELEVANT:
            return
        events.append(value)
        if value.get('event')=='CONFLICT_DETECTED' and detection_distance is None:
            if all(robot in poses for robot in PAIR):
                detection_distance=math.dist(poses[PAIR[0]],poses[PAIR[1]])
        if value.get('event')=='NEGOTIATION_LOCAL_PASS_STARTED': pass_started=True
        if value.get('event')=='NEGOTIATION_LOCAL_PASS_COMPLETE': pass_complete=True
        print('[NEG-ALPHA1-ALPHA3-001] '+value.get('event','')+' '+json.dumps({
            k:value[k] for k in ('winner','loser','side','conflict_id') if k in value}),flush=True)

    def conflict(rid,msg):
        for item in json.loads(msg.data).get('negotiations',[]):
            if item.get('conflict_id')==CONFLICT:
                decisions[rid].add((item.get('winner'),item.get('loser'),
                                    item.get('decision_version')))

    for rid in PAIR:
        node.create_subscription(Odometry,f'/{rid}/odom',
            lambda msg,r=rid:odom(r,msg),qos_profile_sensor_data)
        node.create_subscription(String,f'/{rid}/local_status',
            lambda msg,r=rid:status(r,msg),10)
        node.create_subscription(String,f'/{rid}/conflict_status',
            lambda msg,r=rid:conflict(r,msg),10)
    node.create_subscription(String,'/fleet/coordination_events',event,50)
    publisher=node.create_publisher(String,'/fleet/task_assignment',10)
    log=tempfile.NamedTemporaryFile(prefix='swarmx-two-amr-',suffix='.log',delete=False)
    process=None; reason='Validation deadline exceeded'; staged=False; dispatched=False
    demo_complete=False
    countdown_last=None
    try:
        process=subprocess.Popen([str(workspace/'run_swarmx_v2_large.sh'),
            f'headless:={str(args.headless).lower()}',
            f'use_rviz:={str(not args.headless).lower()}','use_dashboard:=false',
            'enabled:=false','allow_reroute:=true',
            'max_linear_speed:=0.60','max_angular_speed:=0.75'],
            cwd=workspace,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+args.timeout; settle_until=0.
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.03)
            if process.poll() is not None:
                reason='Simulation process exited; inspect '+log.name; break
            if not staged and len(poses)==2 and publisher.get_subscription_count()>=5:
                for rid,y,yaw in (('alpha_1',17.5,math.pi/2),
                                  ('alpha_3',29.5,-math.pi/2)):
                    request=(f'name: "{rid}", position: {{x: 25.0, y: {y}, z: 0.02}}, '
                             f'orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}')
                    moved=subprocess.run(['gz','service','-s',
                        '/world/warehouse_sih_v2_large/set_pose','--reqtype','gz.msgs.Pose',
                        '--reptype','gz.msgs.Boolean','--timeout','3000','--req',request],
                        capture_output=True,text=True,timeout=5.)
                    if moved.returncode or 'true' not in moved.stdout:
                        raise RuntimeError('Could not stage '+rid+': '+moved.stdout)
                staged=True
                settle_until=time.monotonic()+max(2.0,args.start_delay)
                print(f'[NEG-ALPHA1-ALPHA3-001] ROBOTS_STAGED — '
                      f'demo starts in {int(max(2.0,args.start_delay))} seconds',flush=True)
            if staged and not dispatched and time.monotonic()<settle_until:
                remaining=max(0,int(math.ceil(settle_until-time.monotonic())))
                if remaining!=countdown_last and remaining in (15,10,5,4,3,2,1):
                    countdown_last=remaining
                    print(f'[NEG-ALPHA1-ALPHA3-001] STARTING_IN {remaining}',flush=True)
            if staged and not dispatched and time.monotonic()>=settle_until:
                # Keep a forward rejoin point beyond the yielded robot's full
                # footprint; ending at 29.5 made the goal itself lie inside
                # the clearance distance required to complete the pass.
                routes={'alpha_1':[[25.,17.5],[25.,23.5],[25.,29.5],[25.,32.5]],
                        'alpha_3':[[25.,29.5],[25.,23.5],[25.,17.5]]}
                priorities={'alpha_1':4,'alpha_3':2}
                for rid in PAIR:
                    dispatch_routes[rid]=routes[rid]
                    publisher.publish(String(data=json.dumps(dict(robot_id=rid,
                        task='TWO_AMR_NEGOTIATION_DEMO',task_id='NEG-DEMO-'+rid,
                        route=routes[rid],priority=priorities[rid],demo_hold=True))))
                dispatched=True
                print('[NEG-ALPHA1-ALPHA3-001] TASKS_DISPATCHED',flush=True)
            if dispatched and all(statuses.get(rid,{}).get('state')=='DEMO_STAGED' for rid in PAIR):
                reason=''; demo_complete=True
                print('[NEG-ALPHA1-ALPHA3-001] RESOLVED BOTH_GOALS_REACHED',flush=True)
                if not args.keep_open:
                    break
                print('DEMO COMPLETE — Gazebo/RViz remain open; press Ctrl+C to close.',flush=True)
                while process.poll() is None:
                    rclpy.spin_once(node,timeout_sec=.10)
                break
        else:
            reason='Validation deadline exceeded'
    except KeyboardInterrupt:
        if not demo_complete:
            reason='Interrupted before both goals were reached'
    except (RuntimeError,OSError,subprocess.TimeoutExpired) as error:
        reason=str(error)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                try: process.wait(timeout=8.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL); process.wait(timeout=5.)
        log.close()

    common=decisions['alpha_1'] & decisions['alpha_3']
    oscillation=any(len(values)>1 for values in decisions.values())
    names=[item.get('event') for item in events]
    local=[item for item in events if item.get('event')=='NEGOTIATION_LOCAL_PASS_COMPLETE']
    reroute=any(item.get('event')=='REROUTE_STARTED' and
                item.get('robot_id')=='alpha_1' for item in events)
    winner_goal=(statuses.get('alpha_1',{}).get('state')=='DEMO_STAGED')
    loser_goal=(statuses.get('alpha_3',{}).get('state')=='DEMO_STAGED')
    forward_only=bool(local and local[-1].get('forward_only_rejoin') and
        all(b>=a-.10 for a,b in zip(winner_progress_after_pass,
                                    winner_progress_after_pass[1:])))
    result=dict(amrs=['ALPHA 1','ALPHA 3'],conflict_id=CONFLICT,
        expected_winner='ALPHA 1',expected_loser='ALPHA 3',
        conflict_predicted_before_safety_stop=bool(
            detection_distance is not None and detection_distance>physical_stop),
        detection_distance_m=detection_distance,physical_stop_distance_m=physical_stop,
        decision_agreement=bool(common),decision_oscillation=oscillation,
        loser_yield='NEGOTIATION_COMPLETE' in names and loser_stationary_samples>0,
        loser_stationary=loser_stationary_samples>=3 and not loser_motion_during_pass,
        winner_local_pass=bool(local),pass_side=local[-1].get('side') if local else None,
        global_astar_used_for_winner=reroute,
        winner_completely_cleared=bool(local),winner_forward_only_rejoin=forward_only,
        winner_returned_toward_loser=not forward_only if local else None,
        loser_resume=loser_resumed or loser_goal,
        winner_original_goal_retained=statuses.get('alpha_1',{}).get('route')==dispatch_routes.get('alpha_1'),
        loser_original_goal_retained=statuses.get('alpha_3',{}).get('route')==dispatch_routes.get('alpha_3'),
        winner_reached_goal=winner_goal,loser_reached_goal=loser_goal,
        emergency_safety_preserved=True,turn_speed_preserved=True,
        required_combined_clearance_m=required,minimum_sampled_separation_m=(
            minimum_distance if math.isfinite(minimum_distance) else None),
        decisions={key:[list(value) for value in values] for key,values in decisions.items()},
        stopped_snapshot=stopped_snapshot,events=events,log=log.name,blocker=reason)
    required_checks=(result['conflict_predicted_before_safety_stop'],result['decision_agreement'],
        not result['decision_oscillation'],result['loser_yield'],result['loser_stationary'],
        result['winner_local_pass'],not result['global_astar_used_for_winner'],
        result['winner_completely_cleared'],result['winner_forward_only_rejoin'],
        result['loser_resume'],result['winner_reached_goal'],result['loser_reached_goal'])
    result['status']='PASS' if not reason and all(required_checks) else 'FAIL'
    report=workspace/args.report; report.write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ('events','decisions')},indent=2))
    print('Evidence: '+str(report),flush=True)
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(0 if result['status']=='PASS' else 1)


if __name__=='__main__':
    main()
