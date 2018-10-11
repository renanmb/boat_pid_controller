The only launch file that matters is controller.launch


I am trying to launch two nodes: pid_controller.py and throttle_interpolator.py but the ROS keeps giving me back the error: 

> "cannot marshal None unless allow_none
> is enabled ”

The problem might be related to the vesc.yaml, but honestly I do not know anymore. It has been frustrating. 

 
my github: https://github.com/renanmb/boat_pid_controller

I am getting trouble only with:

    <arg name="vesc_config" default="$(find boat_controller)/config/vesc.yaml" />
        	<rosparam file="$(arg vesc_config)" command="load" ns="/"/>
and with

    <node pkg="boat_controller" type="throttle_interpolator.py" name="throttle_interpolator">
        		
        	</node>

when it is commented everything works just fine.

my launch file:

    <?xml version="1.0"?>
    <launch>
    	<arg name="vesc_config" default="$(find boat_controller)/config/vesc.yaml" />
    	<rosparam file="$(arg vesc_config)" command="load" ns="/"/>
    
    	<node pkg="boat_controller" type="pid_controller.py" name="pid_controller" output="screen">
    		  <param name="yawKp" value="0.2" type="double"/>
    		  <param name="yawKi" value="0.0" type="double"/>
    		  <param name="yawKd" value="0.0" type="double"/>
    
    		  <param name="velKp" value="0.7" type="double"/>
    		  <param name="velKi" value="0.3" type="double"/>
    		  <param name="velKd" value="0.0" type="double"/>
    
    		  <param name="start_engaged" value="true" type="bool"/>
    		  <param name="yaw_cntrl" value="true" type="bool"/>
    		  <param name="vel_cntrl" value="true" type="bool"/>
    
    	</node> 
    
    	<node pkg="boat_controller" type="throttle_interpolator.py" name="throttle_interpolator">
    		
    	</node>
    
    </launch>

After trying to run the launch file the terminal outputs this:

    ubuntu-13028.log
    Checking log directory for disk usage. This may take awhile.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is <1GB.
    
    started roslaunch server http://ubuntu:33415/
    
    SUMMARY
    ========
    
    PARAMETERS
     * /commands/motor/speed_left: commands/motor/sp...
     * /commands/motor/speed_right: commands/motor/sp...
     * /commands/motor/unsmoothed_speed: cmd_drive
     * /commands/servo/position: None
     * /commands/servo/unsmoothed_position: None
     * /max_rpm_acceleration: 100
     * /max_servo_speed: 3.2
     * /pid_controller/start_engaged: True
     * /pid_controller/velKd: 0.0
     * /pid_controller/velKi: 0.3
     * /pid_controller/velKp: 0.7
     * /pid_controller/vel_cntrl: True
     * /pid_controller/yawKd: 0.0
     * /pid_controller/yawKi: 0.0
     * /pid_controller/yawKp: 0.2
     * /pid_controller/yaw_cntrl: True
     * /rosdistro: kinetic
     * /rosversion: 1.12.14
     * /rpm_to_erpm_gain: 7
     * /servo_smoother_rate: 75.0
     * /speed_to_erpm_offset: 0.0
     * /steering_angle_to_servo_gain: -1.2135
     * /steering_angle_to_servo_offset: 0.5304
     * /tachometer_ticks_to_meters_gain: 0.00225
     * /throttle_smoother_rate: 100
     * /vesc_driver/brake_max: 200000.0
     * /vesc_driver/brake_min: -20000.0
     * /vesc_driver/current_max: 20.0
     * /vesc_driver/current_min: 0.0
     * /vesc_driver/duty_cycle_max: 0.0
     * /vesc_driver/duty_cycle_min: 0.0
     * /vesc_driver/port: /dev/ttyACM0
     * /vesc_driver/position_max: 0.0
     * /vesc_driver/position_min: 0.0
     * /vesc_driver/servo_max: 0.85
     * /vesc_driver/servo_min: 0.15
     * /vesc_driver/speed_max: 3250
     * /vesc_driver/speed_min: -3250
    
    NODES
      /
        pid_controller (boat_controller/pid_controller.py)
        throttle_interpolator (boat_controller/throttle_interpolator.py)
    
    ROS_MASTER_URI=http://localhost:11311
    
    load_parameters: unable to set parameters (last param was [/vesc_driver/position_max=0.0]): cannot marshal None unless allow_none is enabled
    Traceback (most recent call last):
      File "/opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/__init__.py", line 306, in main
        p.start()
      File "/opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/parent.py", line 279, in start
        self.runner.launch()
      File "/opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/launch.py", line 657, in launch
        self._setup()
      File "/opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/launch.py", line 644, in _setup
        self._load_parameters()
      File "/opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/launch.py", line 338, in _load_parameters
        r  = param_server_multi()
      File "/usr/lib/python2.7/xmlrpclib.py", line 1006, in __call__
        return MultiCallIterator(self.__server.system.multicall(marshalled_list))
      File "/usr/lib/python2.7/xmlrpclib.py", line 1243, in __call__
        return self.__send(self.__name, args)
      File "/usr/lib/python2.7/xmlrpclib.py", line 1596, in __request
        allow_none=self.__allow_none)
      File "/usr/lib/python2.7/xmlrpclib.py", line 1094, in dumps
        data = m.dumps(params)
      File "/usr/lib/python2.7/xmlrpclib.py", line 638, in dumps
        dump(v, write)
      File "/usr/lib/python2.7/xmlrpclib.py", line 660, in __dump
        f(self, value, write)
      File "/usr/lib/python2.7/xmlrpclib.py", line 719, in dump_array
        dump(v, write)
      File "/usr/lib/python2.7/xmlrpclib.py", line 660, in __dump
        f(self, value, write)
      File "/usr/lib/python2.7/xmlrpclib.py", line 741, in dump_struct
        dump(v, write)
      File "/usr/lib/python2.7/xmlrpclib.py", line 660, in __dump
        f(self, value, write)
      File "/usr/lib/python2.7/xmlrpclib.py", line 719, in dump_array
        dump(v, write)
      File "/usr/lib/python2.7/xmlrpclib.py", line 660, in __dump
        f(self, value, write)
      File "/usr/lib/python2.7/xmlrpclib.py", line 664, in dump_nil
        raise TypeError, "cannot marshal None unless allow_none is enabled"
    TypeError: cannot marshal None unless allow_none is enabled

