"""Focused negative safety test; launches and always stops its own V2 stack.

Run from Ubuntu Terminal after sourcing ROS and install/setup.bash:
    python3 validate_v2_occupied_bay.py

ALPHA 5 is deliberately routed toward ALPHA 4's occupied charging bay. PASS
means it physically approaches and remains braked, not that a fleet mission
completes. No live dashboard or existing stack is reused.
"""
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

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def main():
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1',8091))==0:
            raise SystemExit('Refusing to reuse a running V2 dashboard/stack.')
    # Check before launching: no unrelated simulator is stopped by this test.
    running=subprocess.run(['pgrep','-af','[g]z sim'],capture_output=True,text=True)
    if running.returncode==0:
        raise SystemExit('An existing Gazebo process must be stopped first.')
    workspace=Path(__file__).resolve().parent
    poses={}; seen={}; command={}
    minimum=float('inf'); start=None; held_at=None; held_pose=None
    result={'status':'FAIL','test':'occupied_bay_physical_brake'}
    rclpy.init()
    node=rclpy.create_node('v2_occupied_bay_validation')

    def odom(rid,msg):
        poses[rid]=(msg.pose.pose.position.x,msg.pose.pose.position.y)
        seen[rid]=time.monotonic()

    def coordination(msg):
        nonlocal command
        command=json.loads(msg.data)

    for i in range(1,6):
        rid=f'alpha_{i}'
        node.create_subscription(Odometry,f'/{rid}/odom',
            lambda msg,r=rid:odom(r,msg),qos_profile_sensor_data)
    node.create_subscription(String,'/alpha_5/coordination_command',coordination,10)
    publisher=node.create_publisher(String,'/fleet/task_assignment',10)
    log=tempfile.NamedTemporaryFile(prefix='swarmx-v2-occupied-',suffix='.log',delete=False)
    process=None
    try:
        process=subprocess.Popen([str(workspace/'run_swarmx_v2_demo.sh'),
            'headless:=true','enabled:=false','use_rviz:=false'],cwd=workspace,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+90.; progress_at=0.
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            now=time.monotonic()
            if process.poll() is not None:
                raise RuntimeError('Simulation exited before the check completed')
            if len(poses)<5 or publisher.get_subscription_count()<5:
                continue
            if start is None:
                start=poses['alpha_5']
                route=[list(start),list(poses['alpha_4'])]
                publisher.publish(String(data=json.dumps(dict(robot_id='alpha_5',
                    task='OCCUPIED_BAY_SAFETY_TEST',task_id='V2-OCCUPIED-BAY',
                    route=route,priority=4,demo_hold=True))))
            if now-min(seen.values())>2.:
                raise RuntimeError('Physical odometry became stale')
            distance=min(math.dist(a,b) for a,b in itertools.combinations(poses.values(),2))
            minimum=min(minimum,distance)
            if distance<1.5:
                raise RuntimeError('Preemptive safety stop: separation below 1.50 m')
            moved=math.dist(start,poses['alpha_5'])
            if command.get('reason')=='PEER_FOOTPRINT_BLOCKED':
                if held_at is None:
                    held_at=now
                if now-held_at>=1. and held_pose is None:
                    held_pose=poses['alpha_5']
                if now-held_at>=5.:
                    drift=math.dist(held_pose,poses['alpha_5'])
                    if moved<.30 or drift>.08:
                        raise RuntimeError('Approach or stable physical hold not confirmed')
                    result.update(status='PASS',approach_m=round(moved,3),
                        hold_drift_m=round(drift,3),hold_s=round(now-held_at,2))
                    break
            else:
                held_at=None; held_pose=None
            if now>=progress_at:
                print(json.dumps(dict(separation_m=round(distance,3),
                    moved_m=round(moved,3),reason=command.get('reason'))),flush=True)
                progress_at=now+10.
        else:
            raise RuntimeError('Timed out without a verified physical brake')
    except (RuntimeError,KeyboardInterrupt) as error:
        result['reason']=str(error) or 'Interrupted'
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                process.wait(timeout=10.)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    result.update(minimum_sampled_separation_m=round(minimum,3),log=log.name)
    print(json.dumps(result),flush=True)
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())
