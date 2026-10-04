"""Task-only fleet dashboard: observes decentralized bids, claims and execution."""
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from edge_ai_nav.fleet.route_graph import WarehouseGraph


class FleetDashboard(Node):
    def __init__(self):
        super().__init__('fleet_dashboard')
        self.declare_parameter('port', 8090)
        self.declare_parameter('config_file', '')
        self.static_dir = os.path.join(get_package_share_directory('edge_ai_nav'),
                                       'visualization', 'static')
        config_path = (self.get_parameter('config_file').value or
            os.path.join(get_package_share_directory('amr_simulation'),
                         'config', 'warehouse_sih_demo.yaml'))
        with open(config_path) as stream:
            self.config = yaml.safe_load(stream)['warehouse']
        bounds = self.config.get('bounds', {})
        self.v2_map = (bounds.get('x') == [0.0, 50.0] and
                       bounds.get('y') == [0.0, 40.0])
        self.map_layout = (dict(kind='v2', bounds=bounds,
            shelves=self.config.get('shelves', {}),
            pickups=self.config.get('pickups', {}),
            drops=self.config.get('drops', {}),
            charging_station=self.config.get('charging_station', {}),
            charging_bays=self.config.get('charging_bays', {}),
            cross_aisles=self.config.get('cross_aisle_centrelines', []),
            coordination_zones={name: zone for name, zone in
                self.config.get('zones', {}).items()
                if zone.get('coordination_zone')}) if self.v2_map else None)
        palette = ('#ff3b5c', '#3284ff', '#20dc77', '#ff8a2b', '#a96cff')
        battery = self.config.get('battery_initial_percent', {})
        self.robots = {rid: (visual['label'], palette[index % len(palette)],
                             battery.get(rid, 90))
                       for index, (rid, visual) in
                       enumerate(self.config['robot_visuals'].items())}
        self.graph = WarehouseGraph(self.config)
        self.coordination_events = {}
        self.data = {rid: {'robot_id': rid, 'name': name, 'color': color,
                          'status': 'OFFLINE', 'task': 'NONE', 'battery': battery,
                          'battery_state': 'NORMAL', 'is_charging': False,
                          'zone': 'UNKNOWN', 'localization': 'ODOM',
                          'coordination': 'NONE', 'peers': 0, 'progress': 0,
                          'priority': 0, 'destination': 'NONE',
                          'nav_state': 'UNKNOWN',
                          'last_seen': 0, 'active_since': None,
                          'x': 0, 'y': 0, 'yaw': 0, 'route': [],
                          'waypoint': 0, 'parking_target': ''}
                     for rid, (name, color, battery) in self.robots.items()}
        # Session-scoped, human-readable IDs; time seed avoids reusing an ID
        # when only the dashboard is restarted while AMR bidders stay alive.
        self.task_counter = int(time.time()) % 900 + 99
        self.tasks = {}
        self.latest_task_id = None
        self.events = []
        self.seen_coordination_events = set()
        self.metrics = {'conflicts_detected':0,'conflicts_resolved':0,
                        'p2p_negotiations':0,'reroutes':0,'move_aside_actions':0,
                        'deadlocks_prevented':0,'estimated_delay_avoided':0.0,
                        'last_decision_latency_ms':None,'average_decision_latency_ms':None,
                        'last_action_latency_ms':None,'average_action_latency_ms':None}
        self.decision_latencies=[]; self.action_latencies=[]
        self.task_pub = self.create_publisher(String, '/fleet/tasks', 20)
        self.assignment_pub = self.create_publisher(String, '/fleet/task_assignment', 10)
        self.create_subscription(String, '/fleet/task_bids', self.on_bid, 20)
        self.create_subscription(String, '/fleet/task_claims', self.on_claim, 10)
        self.create_subscription(String, '/fleet/mission_routes', self.on_route, 10)
        self.create_subscription(String, '/fleet/coordination_events', self.on_edge_event, 20)
        for rid in self.robots:
            self.create_subscription(String, f'/{rid}/local_status',
                lambda m, r=rid: self.on_status(r, m), 10)
            self.create_subscription(String, f'/{rid}/coordination_command',
                lambda m, r=rid: self.on_coordination(r, m), 10)
            self.create_subscription(String, f'/{rid}/peer_diagnostics',
                lambda m, r=rid: self.on_peers(r, m), 10)
            self.create_subscription(Odometry, f'/{rid}/odom',
                lambda m, r=rid: self.on_odom(r, m), 10)
        self.server = self.make_server(int(self.get_parameter('port').value))
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info('SIH Fleet Command Dashboard: '
                               f'http://localhost:{self.get_parameter("port").value}/fleet')

    @staticmethod
    def decode(msg):
        try:
            value = json.loads(msg.data)
            return value if isinstance(value, dict) else {}
        except (ValueError, TypeError):
            return {}

    def event(self, text):
        self.events.insert(0, {'time': time.strftime('%H:%M:%S'), 'text': text})
        del self.events[40:]

    def on_bid(self, msg):
        bid = self.decode(msg)
        task = self.tasks.get(bid.get('task_id'))
        rid = bid.get('robot_id')
        if not task or rid not in self.robots or rid in task['bids']:
            return
        task['bids'][rid] = bid
        task['available'] = sum(bool(b.get('eligible')) for b in task['bids'].values())
        self.event(f"{self.robots[rid][0]} bid received · score {bid.get('score')}")

    def on_claim(self, msg):
        claim = self.decode(msg)
        task = self.tasks.get(claim.get('task_id'))
        rid = claim.get('winner')
        if not task or rid not in self.robots:
            return
        task.update(status='CLAIMED', winner=rid, display_name=self.robots[rid][0],
                    route=claim.get('route',[]),
                    reason=claim.get('reason', 'Best deterministic suitability score'),
                    battery=claim.get('battery'), distance=claim.get('distance'),
                    score=claim.get('score'), bid_count=claim.get('bid_count', 0),
                    assignment_source='AUTO — DECENTRALIZED')
        if task['route']:
            task.setdefault('original_route',list(task['route']))
        self.data[rid].update(status='ASSIGNED', task=task['task_id'], progress=0)
        self.event(f"DECENTRALIZED CONSENSUS · {task['bid_count']} bids")
        self.event(f"{task['task_id']} claimed by {self.robots[rid][0]}")

    def on_route(self,msg):
        data=self.decode(msg)
        task=self.tasks.get(data.get('task_id'))
        if task and data.get('robot_id')==task.get('winner'):
            if task.get('route'):
                task.setdefault('original_route',list(task['route']))
            elif data.get('route') and int(data.get('route_version',0))<=1:
                task.setdefault('original_route',list(data['route']))
            task['route']=data.get('route',[])
            task['route_version']=data.get('route_version',task.get('route_version',0))

    def on_status(self, rid, msg):
        d, item = self.decode(msg), self.data[rid]
        previous = item['status']
        state = d.get('state', 'UNKNOWN')
        mission_kind=d.get('mission_kind','TASK')
        task_id = d.get('active_task', item['task'])
        item.update(last_seen=time.monotonic(), task=task_id,
                    progress=round(float(d.get('task_progress', 0))),
                    priority=int(d.get('task_priority',item['priority'])),
                    destination=d.get('destination',item['destination']) or 'NONE',
                    nav_state=state,
                    coordination=d.get('coordination_state', item['coordination']),
                    route=d.get('route',item['route']),
                    waypoint=int(d.get('waypoint',item['waypoint'])),
                    parking_target=d.get('parking_target',item['parking_target']))
        profile=d.get('performance_profile') or {}
        item['missions_completed']=int(profile.get('missions_completed',item.get('missions_completed',0)))
        item['route_efficiency']=d.get('route_efficiency',item.get('route_efficiency'))
        item['performance_profile']=profile
        if d.get('battery_simulated'):
            item['battery']=round(float(d.get('battery_percent',
                d.get('battery_pct',item['battery']))),1)
            item['battery_state']=d.get('battery_state',item['battery_state'])
            item['is_charging']=bool(d.get('is_charging',False))
        if previous == 'ASSIGNED' and state == 'DISABLED' and task_id == 'NONE':
            item['task'] = next((t['task_id'] for t in self.tasks.values()
                if t.get('winner')==rid and t['status']=='CLAIMED'), 'NONE')
            return
        item['status'] = ('COMPLETED' if state == 'MISSION_COMPLETE' and task_id != 'NONE' else
                          'CHARGING' if state == 'CHARGING' else
                          'PARKED' if state in ('PARKED','AVAILABLE') else
                          'DOCKING' if state == 'DOCKING' else
                          'RETURNING TO CHARGE' if mission_kind=='CHARGING' and task_id in ('', 'NONE', None) else
                          'RETURNING TO CHARGE' if state in ('GOING_TO_CHARGE','RETURNING_TO_CHARGE') else
                          'REPOSITIONING' if state in ('CLEARING_DROP_ZONE','POST_TASK_REPOSITION','MOVE_ASIDE','REROUTING','YIELD_RELOCATING') else
                          'WAITING' if state in ('COORDINATION_HOLD', 'OBSTACLE_STOP','YIELD_POCKET') else
                          'IDLE' if state in ('DISABLED', 'MISSION_COMPLETE') else 'EXECUTING')
        item['active_since'] = None
        task = self.tasks.get(task_id)
        # Reattach to an already-running mission after a dashboard-only restart.
        # The controller remains authoritative; no assignment is republished.
        if task is None and task_id != 'NONE' and d.get('route'):
            task = dict(task_id=task_id, type='WAREHOUSE_TRANSFER',
                        pickup=d.get('pickup'), destination=d.get('destination'),
                        priority=d.get('task_priority', 3), status='CLAIMED',
                        winner=rid, display_name=self.robots[rid][0], bids={}, available=0,
                        reason='Live mission reattached; original bid metrics unavailable',
                        battery=item['battery'], distance='N/A', score='N/A',
                        route=d['route'])
            self.tasks[task_id] = task
            if self.latest_task_id is None:
                self.latest_task_id = task_id
            if task_id.startswith('TASK-') and task_id[5:].isdigit():
                self.task_counter = max(self.task_counter, int(task_id[5:]))
        if task and d.get('route'):
            if task.get('route'):
                task.setdefault('original_route',list(task['route']))
            elif int(d.get('route_version',0))<=1:
                task.setdefault('original_route',list(d['route']))
            task['route']=d['route']; task['waypoint']=d.get('waypoint',0)
            task['route_version']=d.get('route_version',task.get('route_version',0))
        if task and item['status'] == 'EXECUTING' and task['status'] in ('CLAIMED','BIDDING'):
            task['status'] = 'EXECUTING'
            self.event(f"{self.robots[rid][0]} executing {task_id}")
        if task and item['status'] == 'COMPLETED' and task['status'] != 'COMPLETED':
            task['status'] = 'COMPLETED'
            self.event(f"{task_id} completed by {self.robots[rid][0]}")

    def on_coordination(self, rid, msg):
        d=self.decode(msg); state=d.get('state','NONE')
        self.data[rid]['coordination'] = state
        key=(state,d.get('conflict_id'))
        if self.coordination_events.get(rid)!=key and state!='NONE':
            zone=d.get('zone','').upper()
            label='AISLE RESERVED / PROCEED' if state=='PROCEED' and zone=='NARROW_AISLE_1' else state
            self.event(f'{self.robots[rid][0]} → {label} · {zone}')
            if state in ('YIELD','YIELD_AND_WAIT'): self.event(f'P2P CONFLICT DETECTED · {zone}')
        self.coordination_events[rid]=key

    def robot_name(self, rid):
        if rid in self.robots: return self.robots[rid][0]
        return next((name for name, _, _ in self.robots.values() if name == rid), str(rid))

    def on_edge_event(self,msg):
        d=self.decode(msg); kind=d.get('event','')
        key=d.get('event_id') or f"{kind}:{d.get('robot_id')}:{d.get('timestamp')}"
        if not kind or key in self.seen_coordination_events: return
        self.seen_coordination_events.add(key)
        if kind=='CONFLICT_DETECTED':
            self.metrics['conflicts_detected']+=1
            robots=' ↔ '.join(self.robot_name(r) for r in d.get('robots',[]))
            self.event(f"P2P CONFLICT PREDICTED · {d.get('zone','').upper()} · {robots} · ETA overlap detected")
        elif kind=='NEGOTIATION_COMPLETE':
            self.metrics['p2p_negotiations']+=1
            latency=d.get('decision_latency_ms')
            if latency is not None:
                self.decision_latencies.append(float(latency))
                self.metrics['last_decision_latency_ms']=round(float(latency),3)
                self.metrics['average_decision_latency_ms']=round(
                    sum(self.decision_latencies)/len(self.decision_latencies),3)
            action=d.get('loser_action','YIELD_AND_WAIT')
            if action=='REROUTE': self.metrics['reroutes']+=1
            if d.get('deadlock_prevented'): self.metrics['deadlocks_prevented']+=1
            self.event(f"P2P NEGOTIATION COMPLETE · {self.robot_name(d.get('winner'))} → PROCEED (ORIGINAL A* ROUTE) · {self.robot_name(d.get('loser'))} → {action}")
            basis=str(d.get('decision_basis','')).replace('_',' ')
            if d.get('decision_basis') == 'TASK_PRIORITY':
                labels={1:'LOW', 2:'NORMAL', 3:'HIGH', 4:'URGENT'}
                basis=f"{labels.get(d.get('winner_priority'), 'HIGH')} > {labels.get(d.get('loser_priority'), 'NORMAL')}"
            self.event(f"DECISION BASIS · {basis}")
            if latency is not None:
                self.event(f"EDGE AI DECISION COMMITTED · {float(latency):.3f} ms")
        elif kind=='NEGOTIATION_ACTION_APPLIED':
            latency=d.get('action_latency_ms')
            if latency is not None:
                self.action_latencies.append(float(latency))
                self.metrics['last_action_latency_ms']=round(float(latency),3)
                self.metrics['average_action_latency_ms']=round(
                    sum(self.action_latencies)/len(self.action_latencies),3)
                self.event(f"EDGE AI CONFLICT ACTION · {self.robot_name(d.get('loser'))} → {d.get('action')} · {float(latency):.3f} ms")
        elif kind=='CONFLICT_RESOLVED':
            self.metrics['conflicts_resolved']+=1
            saved=max(0.,float(d.get('time_saved_s',0.)))
            self.metrics['estimated_delay_avoided']=round(self.metrics['estimated_delay_avoided']+saved,2)
            self.event(f"CONFLICT RESOLVED BY LOCAL EDGE COORDINATION · estimated delay avoided {saved:.2f} s")
        elif kind=='MOVE_ASIDE_STARTED':
            self.metrics['move_aside_actions']+=1
            self.event(f"PATH OBSTRUCTION DETECTED · Idle {self.robot_name(d.get('robot_id'))} → MOVE ASIDE · {d.get('parking_target')}")
        elif kind=='MOVE_ASIDE_COMPLETE':
            self.event(f"ROUTE CLEARED · {self.robot_name(d.get('robot_id'))} parked at {d.get('parking_target')}")
        elif kind=='YIELD_POCKET_SELECTED':
            self.event(f"SAFE WAIT POSITION · {self.robot_name(d.get('loser'))} moving aside for {self.robot_name(d.get('winner'))}")
        elif kind=='YIELD_POCKET_CLEARED':
            self.event(f"PATH CLEAR · {self.robot_name(d.get('robot_id'))} resumes original task")
        elif kind=='TASK_COMPLETED':
            self.event(f"TASK COMPLETED · {self.robot_name(d.get('robot_id'))} cleared {d.get('destination')}")
        elif kind=='POST_TASK_REPOSITION':
            self.event(f"POST-TASK REPOSITION · {self.robot_name(d.get('robot_id'))} → {d.get('parking_target')}")
        elif kind=='PARKED':
            self.event(f"{self.robot_name(d.get('robot_id'))} AVAILABLE · {d.get('parking_target')}")
        elif kind=='REROUTE_STARTED':
            self.event(f"EDGE AI REROUTE · {self.robot_name(d.get('robot_id'))} avoided {d.get('zone')} · +{d.get('extra_detour_m') or 0:.2f} m")
        elif kind=='ASTAR_ROUTE_PLANNED':
            self.event(f"A* ROUTE PLANNED · {self.robot_name(d.get('robot_id'))} · expanded {d.get('expanded_nodes',0)} · raw {d.get('raw_astar_nodes',0)} → smooth {d.get('smoothed_waypoints',0)}")
        elif kind=='LIDAR_OBSTACLE_DETECTED':
            self.event(f"LIDAR OBSTACLE DETECTED · {self.robot_name(d.get('robot_id'))} · {d.get('distance_m')} m")
        elif kind=='LOCAL_EDGE_AVOIDANCE_ACTIVE':
            self.event(f"LOCAL EDGE AVOIDANCE ACTIVE · {self.robot_name(d.get('robot_id'))} → {d.get('side')}")
        elif kind=='ORIGINAL_ASTAR_ROUTE_REACQUIRED':
            self.event(f"ORIGINAL A* ROUTE REACQUIRED · {self.robot_name(d.get('robot_id'))}")
        elif kind=='PERSISTENT_BLOCKAGE_DETECTED':
            self.event(f"PERSISTENT AISLE BLOCKAGE DETECTED · {d.get('preset')}")
        elif kind=='ASTAR_REPLANNING_FROM_CURRENT_POSITION':
            self.event(f"A* REPLANNING FROM CURRENT POSITION · blocked {d.get('blocked_zone')}")
        elif kind=='ALTERNATE_ROUTE_ACTIVE':
            self.event(f"ALTERNATE ROUTE ACTIVE · edge avoided {d.get('blocked_zone')} · route v{d.get('route_version')}")
        elif kind=='DROP_ZONE_CLEARING':
            self.event(f"DROP ZONE CLEARING · {self.robot_name(d.get('robot_id'))} leaving destination")
        elif kind in ('CHARGING_BAY_RESERVED','CHARGE_SLOT_RESERVED'):
            self.event(f"CHARGE SLOT RESERVED · {self.robot_name(d.get('robot_id'))} → {d.get('parking_target')}")
        elif kind in ('ASTAR_POST_TASK_ROUTE','ASTAR_RETURN_ROUTE'):
            self.event(f"A* RETURN ROUTE · {self.robot_name(d.get('robot_id'))} → CHARGING STATION · {d.get('parking_target')}")
        elif kind=='DOCKING':
            self.event(f"DOCKING · {self.robot_name(d.get('robot_id'))} → {d.get('parking_target')}")
        elif kind=='CHARGING_STARTED':
            self.event(f"CHARGING STARTED · {self.robot_name(d.get('robot_id'))} at {d.get('parking_target')} · Battery {d.get('battery_percent')}%")
        elif kind=='BATTERY_FULL':
            self.event(f"BATTERY FULL · {self.robot_name(d.get('robot_id'))} · 100%")
        elif kind=='CHARGE_SLOT_RELEASED':
            self.event(f"CHARGE SLOT RELEASED · {self.robot_name(d.get('robot_id'))} ← {d.get('parking_target')}")
        elif kind=='SAFE_WAIT_ROUTE':
            self.event(f"CHARGING FULL · {self.robot_name(d.get('robot_id'))} → SAFE WAIT {d.get('parking_target')}")
        elif kind=='MISSION_PERFORMANCE':
            metrics=d.get('metrics',{})
            self.event(f"PERFORMANCE PROFILE UPDATED · {self.robot_name(d.get('robot_id'))} · route efficiency {100*float(metrics.get('route_efficiency',0)):.1f}%")

    def on_peers(self, rid, msg):
        self.data[rid]['peers'] = int(self.decode(msg).get('count', 0))

    def on_odom(self, rid, msg):
        item = self.data[rid]
        item['x'], item['y'] = round(msg.pose.pose.position.x, 2), round(msg.pose.pose.position.y, 2)
        q = msg.pose.pose.orientation
        item['yaw'] = round(math.atan2(2*(q.w*q.z+q.x*q.y),
                                       1-2*(q.y*q.y+q.z*q.z)), 3)
        x, y = item['x'], item['y']
        if self.v2_map:
            station=self.config['charging_station']
            cx,cy=station['center']; width,height=station['size']
            zone=next((name.replace('_',' ') for name,z in
                self.map_layout['coordination_zones'].items()
                if (math.dist((x,y),z['center']) <= z['radius']
                    if 'radius' in z else
                    z['bounds']['x'][0] <= x <= z['bounds']['x'][1] and
                    z['bounds']['y'][0] <= y <= z['bounds']['y'][1])),None)
            at_charger=(abs(x-cx)<=width/2 and abs(y-cy)<=height/2)
            item['zone']=(zone or ('CHARGING STATION' if at_charger else
                           'MAIN AISLE' if abs(x-cx)<2.0 else 'WAREHOUSE AISLE'))
        else:
            item['zone'] = ('INTERSECTION A' if x*x+y*y < 1.7 else
                            'LOADING BAY' if x < -7 else
                            'NARROW AISLE' if x > 7 and y > 1 else 'MAIN CORRIDOR')

    def create_task(self, payload):
        required = ('type', 'pickup', 'destination', 'priority')
        if not all(payload.get(k) for k in required):
            return {'success':False,'reason':'Missing required task fields'}
        if any(payload[k] not in self.graph.nodes for k in ('pickup','destination')):
            return {'success':False,'reason':'Unknown pickup or destination'}
        mode=str(payload.get('assignment_mode','AUTO')).upper()
        manual_robot=payload.get('robot_id')
        if mode not in ('AUTO','MANUAL'):
            return {'success':False,'reason':'Invalid assignment mode'}
        if mode=='MANUAL':
            if manual_robot not in self.robots:
                return {'success':False,'reason':'Select a valid AMR'}
            robot=self.data[manual_robot]
            if time.monotonic()-robot['last_seen']>3 or robot['status']=='OFFLINE':
                return {'success':False,'reason':f'{self.robots[manual_robot][0]} is OFFLINE'}
            if robot.get('battery_state')=='CRITICAL':
                return {'success':False,'reason':f'{self.robots[manual_robot][0]} has CRITICAL BATTERY'}
            if robot['status'] in ('EXECUTING','ASSIGNED','WAITING','REPOSITIONING'):
                return {'success':False,'reason':f'{self.robots[manual_robot][0]} is already executing a mission'}
            if robot['status']=='FAULTED':
                return {'success':False,'reason':f'{self.robots[manual_robot][0]} is FAULTED'}
            try:
                plan=self.graph.mission_plan((robot['x'],robot['y']),payload['pickup'],payload['destination'])
            except ValueError as error:
                return {'success':False,'reason':f'No safe A* route: {error}'}
        self.task_counter += 1
        task_id = f'TASK-{self.task_counter}'
        task = {'task_id': task_id, 'type': payload['type'],
                'pickup': payload['pickup'], 'destination': payload['destination'],
                'priority': int(payload['priority']), 'timestamp': time.time(),
                'status': 'BIDDING' if mode=='AUTO' else 'CLAIMED',
                'bids': {}, 'available': 0,
                'winner': None if mode=='AUTO' else manual_robot,
                'assignment_source':('AUTO — DECENTRALIZED' if mode=='AUTO'
                                     else 'MANUAL — OPERATOR OVERRIDE')}
        task['route_name'] = 'DYNAMIC_GRAPH'
        task['route'] = [] if mode=='AUTO' else plan['smoothed_waypoints']
        self.tasks[task_id] = task
        self.latest_task_id = task_id
        self.event(f'{task_id} created · {task["pickup"]} → {task["destination"]}')
        if mode=='AUTO':
            self.event('Fleet task published · AMRs evaluating locally')
            self.task_pub.publish(String(data=json.dumps({k: task[k] for k in
                ('task_id', 'type', 'pickup', 'destination', 'priority', 'timestamp')})))
        else:
            robot=self.data[manual_robot]
            task.update(display_name=self.robots[manual_robot][0],
                        reason='Explicit operator override; local safety remains active',
                        battery=robot['battery'],distance=round(
                            self.graph.pickup_distance((robot['x'],robot['y']),task['pickup']),2),
                        score='OVERRIDE',bid_count=0)
            robot.update(status='ASSIGNED',task=task_id,progress=0)
            assignment=dict(robot_id=manual_robot,task=task['type'],task_id=task_id,
                pickup=task['pickup'],destination=task['destination'],priority=task['priority'],
                route=plan['smoothed_waypoints'],planner='A*',
                raw_astar_nodes=plan['raw_astar_nodes'],raw_waypoints=plan['raw_waypoints'],
                expanded_nodes=plan['expanded_nodes'],raw_route_cost=plan['raw_route_cost'],
                assignment_source='MANUAL_OPERATOR_OVERRIDE')
            self.assignment_pub.publish(String(data=json.dumps(assignment)))
            self.event(f"OPERATOR OVERRIDE · {task_id} assigned to {self.robots[manual_robot][0]} · safety layers active")
        return {'success':True,'task_id':task_id,'assignment_mode':mode}

    def snapshot(self):
        now = time.monotonic()
        robots = []
        for item in self.data.values():
            row = dict(item)
            if now-row['last_seen'] > 3:
                row['status'] = 'OFFLINE'
            row.pop('last_seen', None); row.pop('active_since', None)
            robots.append(row)
        latest = self.tasks.get(self.latest_task_id)
        if latest:
            latest = dict(latest)
            latest['bids'] = list(latest['bids'].values())
        coordination = sum(r['coordination'] in ('YIELD','YIELD_AND_WAIT','WAIT_FOR_CLEAR','SAFE_WAIT','YIELD_RELOCATE')
                           for r in robots)
        return {'robots': robots, 'latest_task': latest, 'events': self.events,
                'map_layout': self.map_layout,
                'routes': [{k:t.get(k) for k in ('task_id','winner','route','original_route',
                                                 'waypoint','route_version','status')}
                           for t in self.tasks.values() if t.get('winner') and t['status']!='COMPLETED'],
                'active_robots': sum(r['status'] != 'OFFLINE' for r in robots),
                'active_tasks': sum(t['status'] in ('BIDDING', 'CLAIMED', 'EXECUTING')
                                    for t in self.tasks.values()),
                'completed_tasks': sum(t['status'] == 'COMPLETED' for t in self.tasks.values()),
                'active_conflicts': coordination, 'metrics':dict(self.metrics),
                'system': 'OPERATIONAL' if any(r['status'] != 'OFFLINE' for r in robots)
                          else 'WAITING FOR ROS'}

    def make_server(self, port):
        node = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass
            def send_json(self, value):
                body = json.dumps(value).encode()
                self.send_response(200); self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body))); self.end_headers()
                self.wfile.write(body)
            def do_GET(self):
                if self.path == '/api/locations':
                    return self.send_json(list(node.graph.nodes))
                if self.path == '/api/fleet':
                    return self.send_json(node.snapshot())
                name = 'fleet_dashboard.html' if self.path in ('/fleet', '/fleet/') else self.path.lstrip('/')
                if name not in ('fleet_dashboard.html', 'fleet_dashboard.css', 'fleet_dashboard.js'):
                    return self.send_error(404)
                path = os.path.join(node.static_dir, name)
                if not os.path.isfile(path):
                    return self.send_error(404)
                with open(path, 'rb') as stream:
                    body = stream.read()
                kind = 'text/html' if name.endswith('.html') else ('text/css' if name.endswith('.css') else 'text/javascript')
                self.send_response(200); self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(body))); self.end_headers()
                self.wfile.write(body)
            def do_POST(self):
                if self.path != '/api/tasks':
                    return self.send_error(404)
                try:
                    payload = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))))
                except (ValueError, TypeError):
                    payload = {}
                result = node.create_task(payload)
                self.send_json(result)
        return ThreadingHTTPServer(('0.0.0.0', port), Handler)


def main(args=None):
    rclpy.init(args=args)
    node = FleetDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
