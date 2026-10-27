#!/usr/bin/env python3
"""AUTOMATED String-input integration test in Gazebo, NOT a human webcam demo.

Run robot_behavior_node first, with the arena prepared and recognition node OFF.
This script never publishes cmd_vel; the real behavior node does that. Measures
real ROS odometry, commands, states and hazards; labels all evidence automated.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector
from rcl_interfaces.srv import GetParameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--interaction-only', action='store_true',
                        help='Run selection/switch/stop checks without repeating full trajectories')
    args = parser.parse_args()
    rclpy.init();node = rclpy.create_node('automated_gesture_sim_test')
    pub = node.create_publisher(String, '/detected_gesture', 1)
    state = [{}];velocity = [(0, 0)];pose = [None];trajectory = [];hazards = [];results = []
    stage = ['setup']
    def odometry(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        pose[0] = (p.x, p.y, yaw)
        trajectory.append({'wall': time.monotonic(), 'stage': stage[0], 'pose': pose[0]})
    subs = [node.create_subscription(String, '/behavior_state', lambda m: state.__setitem__(0,json.loads(m.data)),
                                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            node.create_subscription(Odometry, '/odom', odometry, qos_profile_sensor_data),
            node.create_subscription(Twist, '/cmd_vel', lambda m: velocity.__setitem__(0,(m.linear.x,m.angular.z)),
                                     qos_profile_sensor_data),
            node.create_subscription(HazardDetectionVector, '/hazard_detection',
                                     lambda m: hazards.extend(h.type for h in m.detections),qos_profile_sensor_data)]
    def pump(seconds, gesture=None):
        end=time.monotonic()+seconds;sent=0
        while time.monotonic()<end:
            now=time.monotonic()
            if gesture is not None and now-sent>=.05:pub.publish(String(data=gesture));sent=now
            rclpy.spin_once(node,timeout_sec=.01)
    def check(condition, message):
        if not condition:raise AssertionError(message+': '+str(state[0]))
    def select_and_confirm(gesture):
        pump(.5,gesture)
        check(state[0].get('mode')=='SELECTED','Selection failed')
        check(all(abs(x)<.001 for x in velocity[0]),'Robot moved BEFORE confirmation')
        pump(.5,'Thumb_Up')
        check(state[0].get('mode')=='RUNNING','Confirmation did not start motion')
    def complete(gesture,name):
        stage[0]=name;start=pose[0];t=time.monotonic();select_and_confirm(gesture)
        while time.monotonic()-t<175 and state[0].get('mode')=='RUNNING':pump(.1,'Thumb_Up')
        check(state[0].get('mode')=='COMPLETED',name+' did not complete')
        expected_segments={'shuttle_4_cycles':8,'one_circle':1,'greeting':3}[name]
        check(state[0].get('segment')==expected_segments,name+' completed wrong number of segments')
        end=pose[0];pump(.8,'Thumb_Up')
        check(state[0].get('mode')=='COMPLETED','Held Thumb_Up retriggered behavior')
        displacement=math.hypot(end[0]-start[0],end[1]-start[1])
        points=[s['pose'] for s in trajectory if s['stage']==name]
        turn=sum(math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2])) for a,b in zip(points,points[1:]))
        if name=='shuttle_4_cycles':
            check(displacement<.05,'Shuttle did not return near its initial position')
            distances=[math.hypot(p[0]-start[0],p[1]-start[1]) for p in points]
            check(max(distances)>profile['shuttle_distance']-.08,'Shuttle amplitude too small')
        elif name=='one_circle':
            check(displacement<.10 and abs(turn-2*math.pi)<.15,'Circle closure/heading outside tolerance')
        else:
            check(displacement<.03 and abs(turn)<.15,'Greeting moved/failed to restore heading')
        results.append({'test':name,'passed':True,'elapsed_wall_s':round(time.monotonic()-t,3),
                        'start_pose':start,'end_pose':end,
                        'end_displacement_m':displacement,'yaw_change_rad':turn,
                        'motion_segments_completed':expected_segments})
        print(name,'PASSED',results[-1],flush=True)
    ok=False;profile={}
    try:
        print('AUTOMATED ROS/Gazebo test, not a webcam gesture demonstration.',flush=True)
        end=time.monotonic()+35
        while time.monotonic()<end and (not state[0].get('ready') or pose[0] is None):pump(.1)
        check(state[0].get('ready'),'Sensors/undocked state not ready')
        others=[p.node_name for p in node.get_publishers_info_by_topic('/detected_gesture')
                if p.node_name!='automated_gesture_sim_test']
        check(not others,'Turn OFF webcam recognizer for controlled test: '+str(others))
        client=node.create_client(GetParameters,'/robot_behavior_node/get_parameters')
        check(client.wait_for_service(timeout_sec=5),'Behavior parameters unavailable')
        req=GetParameters.Request();req.names=['linear_speed','angular_speed','circle_speed',
            'shuttle_distance','circle_radius','signal_timeout','selection_signal_timeout']
        future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        check(future.done() and future.result() is not None,'Could not read active controller parameters')
        profile={k:v.double_value for k,v in zip(req.names,future.result().values)}
        pump(.5,'Thumb_Up');check(state[0].get('mode')!='RUNNING','Thumb_Up alone started motion')
        stage[0]='selection_handover'
        pump(.5,'Pointing_Up');pump(.15,'Closed_Fist')
        check(state[0].get('mode')=='SELECTED','Brief transition fist erased choice')
        pump(.9,'No_Hand')
        check(state[0].get('mode')=='SELECTED','Stationary handover gap erased choice')
        check(all(abs(x)<.001 for x in velocity[0]),'Moved during stationary handover')
        pump(.4,'Closed_Fist');check(state[0].get('mode')=='IDLE','Deliberate fist failed to cancel choice')
        pump(.5,'Pointing_Up');pump(2.05,'No_Hand')
        check(state[0].get('mode')=='FAULT','Long selection signal loss failed to clear choice')
        pump(.5,'Pointing_Up');check(state[0].get('mode')=='SELECTED','Same gesture could not reselect after fault')
        pump(.4,'Closed_Fist')
        results.append({'test':'selection_handover_and_rearm','passed':True})
        print('selection_handover_and_rearm PASSED',flush=True)
        stage[0]='switch_behavior';select_and_confirm('Pointing_Up')
        pump(.5,'Pointing_Up');check(state[0].get('mode')=='RUNNING','Current choice restarted/stopped motion')
        for gesture,choice in [('Victory','circle'),('Pointing_Up','shuttle')]:
            pump(.5,gesture)
            check(state[0].get('mode')=='SELECTED' and state[0].get('selected')==choice,
                  'New choice did not replace running behavior')
            check(all(abs(x)<.001 for x in velocity[0]),'Old motion continued after new selection')
            pump(.5,'Thumb_Up');check(state[0].get('mode')=='RUNNING','New choice did not run after fresh confirmation')
        pump(.4,'Closed_Fist')
        results.append({'test':'switch_requires_fresh_confirmation','passed':True})
        print('switch_requires_fresh_confirmation PASSED',flush=True)
        if not args.interaction_only:
            complete('Pointing_Up','shuttle_4_cycles')
            complete('Victory','one_circle')
            complete('Wave','greeting')
        for gesture,expected,name in [('Open_Palm','STOPPED','palm_stop'),
                                      ('Closed_Fist','IDLE','fist_cancel')]:
            stage[0]=name;select_and_confirm('Pointing_Up');pump(.3,'Thumb_Up');pump(.2,gesture)
            check(state[0].get('mode')==expected,name+' state wrong')
            check(all(abs(x)<.001 for x in velocity[0]),name+' velocity not zero')
            results.append({'test':name,'passed':True});print(name,'PASSED',flush=True)
        stage[0]='signal_loss';select_and_confirm('Pointing_Up');pump(.3,'Thumb_Up');pump(.95)
        check(state[0].get('mode')=='FAULT','Signal loss did not fault')
        check(all(abs(x)<.001 for x in velocity[0]),'Signal loss velocity not zero')
        results.append({'test':'signal_loss','passed':True})
        check(not any(h in (1,2,3,4,5) for h in hazards),'Hazard during controlled test')
        ok=True;print('ALL AUTOMATED SIMULATION CHECKS PASSED',flush=True)
    finally:
        pump(.2,'Closed_Fist')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps({'method':'AUTOMATED /detected_gesture String inputs; not live webcam',
            'all_passed':ok,'tests':results,'trajectory':trajectory,'hazard_types':hazards,
            'last_state':state[0], 'active_controller_parameters':profile,
            'scope':'interaction_only' if args.interaction_only else 'full_navigation_and_interaction'},indent=2))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
