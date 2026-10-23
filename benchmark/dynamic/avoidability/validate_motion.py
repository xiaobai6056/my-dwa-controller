#!/usr/bin/env python3
"""Gazebo-only prescribed controls. Not Nav2 success-rate data; no controller plugin loaded."""
import argparse,json,math,os,signal,subprocess,sys,time
from pathlib import Path
from types import SimpleNamespace
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent));sys.path.insert(0,str(HERE.parent/'timing_sweep'))
from motion import Motion
from rules import scene,verify_motion
import rclpy
from gazebo_msgs.msg import ModelStates,ContactsState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data

def main():
    p=argparse.ArgumentParser();p.add_argument('--start',type=float,required=True);p.add_argument('--brake',type=float,required=True);p.add_argument('--run-id',required=True);p.add_argument('--domain',default=92,type=int);a=p.parse_args()
    out=HERE/'runs'/a.run_id;out.mkdir(parents=True,exist_ok=False)
    env=dict(os.environ,ROS_DOMAIN_ID=str(a.domain),GAZEBO_MASTER_URI=f'http://127.0.0.1:{11345+a.domain}')
    os.environ.update(ROS_DOMAIN_ID=env['ROS_DOMAIN_ID'],GAZEBO_MASTER_URI=env['GAZEBO_MASTER_URI'])
    log=(out/'process.log').open('w');streams={k:(out/(k+'.jsonl')).open('w') for k in ('ground_truth','contacts','commands','motion','dynamic_events','odom')}
    raw={k:[] for k in streams}
    def emit(k,v):raw[k].append(v);streams[k].write(json.dumps(v)+'\n')
    procs=[];manifest=dict(classification='gazebo_only_counterfactual_validation',arguments=vars(a),nav2_action_result=None,not_benchmark_success_rate=True)
    c=None
    try:
        subprocess.run([sys.executable,'/home/user/ros2_ws/scripts/_baseline_processes.py'],env=env,stdout=log,stderr=log,check=True,timeout=20)
        world=HERE.parent/'assets/D01/world.sdf';sdf=Path('/opt/ros/humble/share/nav2_bringup/worlds/waffle.model')
        import hashlib
        manifest.update(world=str(world),world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),robot_sdf=str(sdf),robot_sha256=hashlib.sha256(sdf.read_bytes()).hexdigest())
        procs.append(subprocess.Popen(['gzserver','-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so',str(world)],env=env,stdout=log,stderr=log,start_new_session=True))
        rclpy.init();node=rclpy.create_node('isolated_braking_validation')
        c=SimpleNamespace(node=node,clock=SimpleNamespace(sim=None),robot=None,twist=None)
        def clock(m):c.clock.sim=m.clock.sec+m.clock.nanosec*1e-9
        def gt(m):
            if c.clock.sim is None:return
            try:i=m.name.index('turtlebot3_waffle');j=m.name.index('dynamic_obstacle')
            except ValueError:return
            r=m.pose[i];o=m.pose[j];q=r.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));tw=m.twist[i]
            c.robot=[r.position.x,r.position.y,yaw];c.twist=[math.cos(yaw)*tw.linear.x+math.sin(yaw)*tw.linear.y,tw.angular.z]
            emit('ground_truth',dict(t=c.clock.sim,wall_ns=time.monotonic_ns(),robot=c.robot,obstacle=[o.position.x,o.position.y,0],vx=c.twist[0],wz=c.twist[1]))
        def contact(m):
            for s in m.states:
                pair=[s.collision1_name,s.collision2_name]
                if any('turtlebot3_waffle::' in z for z in pair):emit('contacts',dict(t=c.clock.sim,stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,pair=pair,depths=list(s.depths)))
        def odom(m):emit('odom',dict(t=c.clock.sim,stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,x=m.pose.pose.position.x,y=m.pose.pose.position.y,vx=m.twist.twist.linear.x,wz=m.twist.twist.angular.z))
        subs=[node.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data),node.create_subscription(ModelStates,'/benchmark/model_states',gt,qos_profile_sensor_data),node.create_subscription(ContactsState,'/benchmark/contacts',contact,qos_profile_sensor_data),node.create_subscription(Odometry,'/odom',odom,100)]
        pub=node.create_publisher(Twist,'/cmd_vel',10)
        procs.append(subprocess.Popen(['ros2','run','gazebo_ros','spawn_entity.py','-entity','turtlebot3_waffle','-file',str(sdf),'-x','0','-y','0','-z','.01'],env=env,stdout=log,stderr=log,start_new_session=True))
        deadline=time.monotonic()+60
        motion=Motion(c,scene(a.start),emit)
        while c.robot is None or c.clock.sim is None or not motion.client.service_is_ready() or not pub.get_subscription_count():
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()>deadline:raise RuntimeError('Gazebo readiness timeout')
        for _ in range(100):rclpy.spin_once(node,timeout_sec=.005)
        endpoints=node.get_publishers_info_by_topic('/cmd_vel');manifest['cmd_vel_publishers']=[e.node_namespace+'/'+e.node_name for e in endpoints]
        if len(endpoints)!=1 or endpoints[0].node_name!='isolated_braking_validation':raise RuntimeError('control isolation failed')
        manifest['node_names']=node.get_node_names_and_namespaces()
        if any(any(s in n for s in ('controller_server','velocity_smoother','bt_navigator')) for n,_ in manifest['node_names']):raise RuntimeError('unexpected Nav2')
        epoch=c.clock.sim;motion.begin(epoch);manifest['epoch']=epoch;manifest['start_pose']=c.robot
        lastwall=time.monotonic();lastsim=c.clock.sim;v=0.;braked=False;resumed=False;finished=False;deadline=time.monotonic()+150
        while not finished:
            rclpy.spin_once(node,timeout_sec=.002);motion.tick();t=c.clock.sim-epoch
            if motion.error:raise RuntimeError(motion.error)
            now=time.monotonic()
            if now-lastwall>=.05:
                dt=min(.1,max(0,c.clock.sim-lastsim));lastsim=c.clock.sim;lastwall=now
                if t>=a.brake and not braked:
                    manifest['brake_event']=dict(relative_s=t,sim=c.clock.sim,robot=c.robot,vx=c.twist[0]);braked=True
                if t>=a.start+9.3:resumed=True
                target=0. if braked and not resumed else .26
                if c.robot[0]>=3.95:target=0.;finished=abs(c.twist[0])<.01
                v+=max(-2.5*dt,min(2.5*dt,target-v))
                msg=Twist();msg.linear.x=v;pub.publish(msg)
                emit('commands',dict(t=c.clock.sim,relative_s=t,wall_ns=time.monotonic_ns(),vx=v,wz=0.,phase='resume' if resumed else ('brake_wait' if braked else 'approach')))
            if raw['contacts']:
                manifest['termination']='physical_contact';break
            if t>35 or time.monotonic()>deadline:raise RuntimeError('validation timeout')
        for _ in range(6):pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.02)
        manifest.setdefault('termination','controlled_goal_region_no_contact')
        manifest['end_relative_s']=c.clock.sim-epoch
        manifest['motion_validation']=verify_motion([z for z in raw['ground_truth'] if z['t']>=epoch],raw['motion'],scene(a.start),epoch)
        manifest['valid']=manifest['motion_validation']['valid']
        if not manifest['valid']:raise RuntimeError('actual obstacle motion verification failed')
    except Exception as exc:manifest.update(valid=False,error=repr(exc));raise
    finally:
        if c is not None:
            try:c.node.destroy_node();rclpy.shutdown()
            except Exception:pass
        for proc in reversed(procs):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        cleanup=subprocess.run([sys.executable,'/home/user/ros2_ws/scripts/_baseline_processes.py'],env=env,stdout=log,stderr=log,timeout=20)
        manifest['cleanup_returncode']=cleanup.returncode
        for f in streams.values():f.close()
        log.close();(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        print(json.dumps(manifest,indent=2),flush=True)
if __name__=='__main__':main()