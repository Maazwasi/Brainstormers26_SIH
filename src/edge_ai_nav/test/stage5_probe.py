"""Bounded passive ROS evidence capture; never publishes control commands."""
import json
import sys
import time
import rclpy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool

rclpy.init()
node = rclpy.create_node('stage5_evidence_probe')
latest, events, positions, clocks = {}, [], {}, []

def status(msg, rid):
    data = json.loads(msg.data)
    latest[rid] = data
    if data['active_conflicts'] and not any(e['robot_id']==rid for e in events): events.append(data)

def packet(msg):
    data=json.loads(msg.data)
    rid=data['robot_id']
    if rid not in positions: positions[rid]={'first':data}
    positions[rid]['last']=data

for rid in ('alpha','bravo','charlie','delta','echo'):
    node.create_subscription(String, '/amr_'+rid+'/conflict_status', lambda msg, rid=rid:status(msg,rid.upper()),10)
node.create_subscription(String,'/fleet/peer_state',packet,10)
node.create_subscription(Clock,'/clock',lambda msg:clocks.append((time.monotonic(),msg.clock.sec+msg.clock.nanosec/1e9)),10)
ready_deadline = time.monotonic()+30
while len(positions)<5 and time.monotonic()<ready_deadline:
    rclpy.spin_once(node,timeout_sec=.1)
assert len(positions)==5, 'No complete fleet data; simulation startup not valid'
if '--enable-pair' in sys.argv:
    for rid in ('alpha','bravo'):
        client=node.create_client(SetBool,'/amr_'+rid+'/autonomy_enabled')
        assert client.wait_for_service(timeout_sec=25)
        future=client.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        assert future.done() and future.result().success
deadline=time.monotonic()+float(sys.argv[1] if len(sys.argv)>1 else 12)
while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
pubs=node.get_publishers_info_by_topic('/fleet/peer_state')
subs=node.get_subscriptions_info_by_topic('/fleet/peer_state')
result={'tables':latest,'predictions':events,'packets':positions,
 'publishers':len(pubs),'peer_subscribers':len([s for s in subs if s.node_name=='peer_state']),
 'peer_node_publishers':{rid:node.get_publisher_names_and_types_by_node('peer_state','/amr_'+rid)
     for rid in ('alpha','bravo','charlie','delta','echo')},
 'rtf':(clocks[-1][1]-clocks[0][1])/(clocks[-1][0]-clocks[0][0]) if len(clocks)>1 else None}
print(json.dumps(result,indent=2))
node.destroy_node()
rclpy.shutdown()
