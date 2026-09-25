"""Integration test supervisor, not a fleet node: enables safe hold-point missions.

Fault-injects only BRAVO's peer process; detectors own all conflict decisions.
"""
import json
import os
import signal
import subprocess
import sys
import time
import rclpy
from std_msgs.msg import String
from std_srvs.srv import SetBool

rclpy.init()
node = rclpy.create_node('stage5_test_supervisor')
states, peers, nav = {}, {}, {}
records=[]

def receive(msg, rid):
    data=json.loads(msg.data); states[rid]=data
    if data['active_conflicts']: records.append(data)

for rid in ('alpha','bravo','charlie','delta','echo'):
    node.create_subscription(String, '/amr_'+rid+'/conflict_status',lambda msg,rid=rid:receive(msg,rid),10)
    node.create_subscription(String, '/amr_'+rid+'/local_status',lambda msg,rid=rid:nav.update({rid:json.loads(msg.data)}),10)
node.create_subscription(String,'/fleet/peer_state',lambda msg:peers.update({json.loads(msg.data)['robot_id']:json.loads(msg.data)}),10)

def until(predicate, seconds=20):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.05)
        if predicate(): return True
    return False

def enable(rid, value):
    client=node.create_client(SetBool,'/amr_'+rid+'/autonomy_enabled')
    if not client.wait_for_service(timeout_sec=15): raise RuntimeError('controller missing: '+rid)
    future=client.call_async(SetBool.Request(data=value))
    if not until(future.done): raise RuntimeError('enable timeout')
    assert future.result().success

result={}
stopped=None
try:
    assert until(lambda:len(states)==5 and len(peers)==5,30)
    result['disabled_false_conflicts']={rid:s['active_conflicts'] for rid,s in states.items()}
    enable('alpha',True); enable('bravo',True)
    assert until(lambda:states['alpha']['active_conflicts'] and states['bravo']['active_conflicts'],20)
    result['bilateral']={rid:states[rid] for rid in ('alpha','bravo')}
    result['moving']={rid:nav[rid] for rid in ('alpha','bravo')}
    result['other_detectors']={rid:states[rid] for rid in ('charlie','delta','echo')}
    # Exact peer executable + namespace; never stop the controller or Gazebo.
    rows=subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines()
    matches=[int(row.split()[0]) for row in rows if '/lib/edge_ai_nav/peer_state_node ' in row and '__ns:=/amr_bravo ' in row]
    assert len(matches)==1, matches
    if '--clear-only' not in sys.argv:
        stopped=matches[0]; os.kill(stopped,signal.SIGSTOP)
        assert until(lambda:states['alpha']['active_conflicts']==0,5)
        result['peer_loss_alpha']=states['alpha']
        os.kill(stopped,signal.SIGCONT); stopped=None
        assert until(lambda:states['alpha']['active_conflicts'] and states['bravo']['active_conflicts'],10)
    # Explicitly withdraw mission intent through the existing enable service.
    enable('bravo',False)
    assert until(lambda:states['alpha']['active_conflicts']==0 and states['bravo']['active_conflicts']==0,10)
    result['cleared']={rid:states[rid] for rid in ('alpha','bravo')}
    result['pass']=True
except Exception as exc:
    result.update(error=repr(exc),last_states=states,pass_=False)
finally:
    if stopped: os.kill(stopped,signal.SIGCONT)
    for rid in ('alpha','bravo'): enable(rid,False)
    print(json.dumps(result,indent=2),flush=True)
    node.destroy_node(); rclpy.shutdown()
