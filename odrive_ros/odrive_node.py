
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import Executor, SingleThreadedExecutor, MultiThreadedExecutor

from typing import Optional

# import tf.transformations
# import tf_conversions
import tf2_ros

import odrive_ros.transformations as transformations

import std_msgs.msg
from std_msgs.msg import Float64, Int32, String
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import std_srvs.srv

# looks like this isn't quite ready for eloquent?
import diagnostic_updater, diagnostic_msgs.msg

import time
import math
import traceback
import queue
from queue import Queue

from .odrive_interface import ODriveInterfaceAPI, ODriveFailure
from .odrive_interface import ChannelBrokenException, ChannelDamagedException
from .odrive_simulator import ODriveInterfaceSimulator

class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def __init__(self, ros_node):
        self.node = ros_node

    def debug(self, msg):    self.node._logger.debug(msg)
    def info(self, msg):     self.node._logger.info(msg)
    def warn(self, msg):     self.node._logger.warn(msg)
    def error(self, msg):    self.node._logger.error(msg)
    def critical(self, msg): self.node._logger.fatal(msg)
    
    # use_index = False (bool)
    # offset_float = 0.590887010098 (float)
    # calib_range = 0.019999999553 (float)
    # mode = 0 (int)
    # offset = 1809 (int)
    # cpr = 4096 (int)
    # idx_search_speed = 10.0 (float)
    # pre_calibrated = False (bool)

#m_s_to_rpm = 60.0/tyre_circumference
#m_s_to_erpm = 10 * m_s_to_rpm 

# 4096 counts / rev, so 4096 == 1 rev/s


# 1 m/s = 3.6 km/hr


class ODriveNode(Node):
    
    def __init__(self):
        super().__init__('odrive_node', namespace="", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self._logger.info("Odrive Node Init")
        # print(self._parameters)
        # print(self.get_parameter('serial_number')._value)

        self.last_speed = 0.0
        self.driver = None
        self.prerolling = False
        
        # Robot wheel_track params for velocity -> motor speed conversion
        self.wheel_track = None
        self.tyre_circumference = None
        self.encoder_counts_per_rev = None
        self.m_s_to_value = 1.0
        self.axis_for_right = 0
        self.encoder_cpr = 4096
        
        # Startup parameters
        self.connect_on_startup = False
        self.calibrate_on_startup = False
        self.engage_on_startup = False
        
        self.publish_joint_angles = True
        # Simulation mode
        # When enabled, output simulated odometry and joint angles (TODO: do joint angles anyway from ?)
        self.sim_mode = False

        try:
            self.sim_mode             = self.better_get_parameter_or('simulation_mode', False).value
            self.serial_number        = self.better_get_parameter_or('serial_number', None).value
            self.publish_joint_angles = self.better_get_parameter_or('publish_joint_angles', True).value # if self.sim_mode else False
            self.publish_temperatures = self.better_get_parameter_or('publish_temperatures', True).value
            
            self.axis_for_right = self.better_get_parameter_or('axis_for_right', 0).value  # if right calibrates first, this should be 0, else 1
            self.wheel_track = self.better_get_parameter_or('wheel_track', 0.285).value  # m, distance between wheel centres
            self.tyre_circumference = self.better_get_parameter_or('tyre_circumference', 0.341).value  # used to translate velocity commands in m/s into motor rpm
            
            self.connect_on_startup   = self.better_get_parameter_or('connect_on_startup', False).value
            self.calibrate_on_startup = self.better_get_parameter_or('calibrate_on_startup', False).value
            self.engage_on_startup    = self.better_get_parameter_or('engage_on_startup', False).value
            
            self.has_preroll     = self.better_get_parameter_or('use_preroll', False).value
            
            self.publish_current = self.better_get_parameter_or('publish_current', True).value
            self.publish_raw_odom = self.better_get_parameter_or('publish_raw_odom', True).value
            
            self.publish_odom    = self.better_get_parameter_or('publish_odom', False).value
            self.publish_tf      = self.better_get_parameter_or('publish_odom_tf', False).value
            self.odom_topic      = self.better_get_parameter_or('odom_topic', "odom").value
            self.odom_frame      = self.better_get_parameter_or('odom_frame', "odom").value
            self.base_frame      = self.better_get_parameter_or('base_frame', "base_link").value
            self.odom_calc_hz    = self.better_get_parameter_or('odom_calc_hz', 10).value
            
            rclpy.get_default_context().on_shutdown(self.terminate)

            
            self.create_service(std_srvs.srv.Trigger, 'connect_driver', self.connect_driver)
            self.create_service(std_srvs.srv.Trigger, 'disconnect_driver', self.disconnect_driver)
            self.create_service(std_srvs.srv.Trigger, 'calibrate_motors', self.calibrate_motor)
            self.create_service(std_srvs.srv.Trigger, 'engage_motors', self.engage_motor)
            self.create_service(std_srvs.srv.Trigger, 'release_motors', self.release_motor)
            
            # odometry update, disable during preroll, whenever wheels off ground 
            self.odometry_update_enabled = True
            self.create_service(std_srvs.srv.SetBool, 'enable_odometry_updates', self.enable_odometry_update_service)
            
            self.status_pub = self.create_publisher(String, 'status', qos_profile=10)
            self.status = "disconnected"
            self.status_pub.publish(String(data=self.status))
            
            self.command_queue = Queue(maxsize=5)
            self.vel_subscribe = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, qos_profile=10)
            
            self.publish_diagnostics = False
            if self.publish_diagnostics:
                self.get_logger().warn("Skipping diagnostics until eloquent release is available")
                self.diagnostic_updater = diagnostic_updater.Updater(self, period=1.0)
                self.diagnostic_updater.setHardwareID("Not connected, unknown")
                self.diagnostic_updater.add("ODrive Diagnostics", self.pub_diagnostics)
            
            if self.publish_temperatures:
                self.temperature_publisher_left  = self.create_publisher(Float64, 'left/temperature', qos_profile=10)
                self.temperature_publisher_right = self.create_publisher(Float64, 'right/temperature', qos_profile=10)
            
            self.i2t_error_latch = False
            if self.publish_current:
                #self.current_loop_count = 0
                #self.left_current_accumulator  = 0.0
                #self.right_current_accumulator = 0.0
                self.current_publisher_left  = self.create_publisher(Float64, 'left/current', qos_profile=10)
                self.current_publisher_right = self.create_publisher(Float64, 'right/current', qos_profile=10)
                self.i2t_publisher_left  = self.create_publisher(Float64, 'left/i2t', qos_profile=10)
                self.i2t_publisher_right = self.create_publisher(Float64, 'right/i2t', qos_profile=10)
                
                self.get_logger().debug("ODrive will publish motor currents.")
                
                self.i2t_resume_threshold  = self.get_parameter_or('i2t_resume_threshold',  222)            
                self.i2t_warning_threshold = self.get_parameter_or('i2t_warning_threshold', 333)
                self.i2t_error_threshold   = self.get_parameter_or('i2t_error_threshold',   666)
            
            self.last_cmd_vel_time = self._clock.now()
            
            if self.publish_raw_odom:
                self.raw_odom_publisher_encoder_left  = self.create_publisher(Int32, 'left/raw_odom/encoder', qos_profile=10) if self.publish_raw_odom else None
                self.raw_odom_publisher_encoder_right = self.create_publisher(Int32, 'right/raw_odom/encoder', qos_profile=10) if self.publish_raw_odom else None
                self.raw_odom_publisher_vel_left      = self.create_publisher(Int32, 'left/raw_odom/velocity', qos_profile=10) if self.publish_raw_odom else None
                self.raw_odom_publisher_vel_right     = self.create_publisher(Int32, 'right/raw_odom/velocity', qos_profile=10) if self.publish_raw_odom else None
                                
            if self.publish_odom:
                self.create_service(std_srvs.srv.Trigger, 'reset_odometry', self.reset_odometry)
                self.old_pos_l = 0
                self.old_pos_r = 0
                
                self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, qos_profile=10)
                # setup message
                self.odom_msg = Odometry()
                #print(dir(self.odom_msg))
                self.odom_msg.header.frame_id = self.odom_frame
                self.odom_msg.child_frame_id = self.base_frame
                self.odom_msg.pose.pose.position.x = 0.0
                self.odom_msg.pose.pose.position.y = 0.0
                self.odom_msg.pose.pose.position.z = 0.0    # always on the ground, we hope
                self.odom_msg.pose.pose.orientation.x = 0.0 # always vertical
                self.odom_msg.pose.pose.orientation.y = 0.0 # always vertical
                self.odom_msg.pose.pose.orientation.z = 0.0
                self.odom_msg.pose.pose.orientation.w = 1.0
                self.odom_msg.twist.twist.linear.x = 0.0
                self.odom_msg.twist.twist.linear.y = 0.0  # no sideways
                self.odom_msg.twist.twist.linear.z = 0.0  # or upwards... only forward
                self.odom_msg.twist.twist.angular.x = 0.0 # or roll
                self.odom_msg.twist.twist.angular.y = 0.0 # or pitch... only yaw
                self.odom_msg.twist.twist.angular.z = 0.0
                
                # store current location to be updated. 
                self.x = 0.0
                self.y = 0.0
                self.theta = 0.0
                
                # setup transform
                self.tf_publisher = tf2_ros.TransformBroadcaster(self)
                self.tf_msg = TransformStamped()
                self.tf_msg.header.frame_id = self.odom_frame
                self.tf_msg.child_frame_id  = self.base_frame
                self.tf_msg.transform.translation.x = 0.0
                self.tf_msg.transform.translation.y = 0.0
                self.tf_msg.transform.translation.z = 0.0
                self.tf_msg.transform.rotation.x = 0.0
                self.tf_msg.transform.rotation.y = 0.0
                self.tf_msg.transform.rotation.w = 0.0
                self.tf_msg.transform.rotation.z = 1.0
                
            if self.publish_joint_angles:
                self.joint_state_publisher = self.create_publisher(JointState, '/odrive/joint_states', qos_profile=10)
                
                jsm = JointState()
                self.joint_state_msg = jsm
                #jsm.name.resize(2)
                #jsm.position.resize(2)
                jsm.name = ['joint_left_wheel','joint_right_wheel']
                jsm.position = [0.0, 0.0]
            
            self.fast_timer = None
            self.vel_l = 0
            self.vel_r = 0
            self.new_pos_l = 0
            self.new_pos_r = 0
            self.current_l = 0
            self.current_r = 0
            self.temp_v_l = 0.
            self.temp_v_r = 0.
            self.motor_state_l = "not connected" # undefined
            self.motor_state_r = "not connected"
            self.bus_voltage = 0.
        except Exception as exc:
            raise exc

        # start things up
        self._logger.info("starting timers")
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        # print("creating rate")
        # main_rate = self.create_rate(10) # hz
        
        # Start timer to run high-rate comms
        print("creating fast timer")
        self.fast_timer = self.create_timer(1/float(self.odom_calc_hz), self.fast_timer_cb)
        
        self.fast_timer_comms_active = False
        
        self.loop_timer = self.create_timer(0.25, self.loop_cb)

    def better_get_parameter_or(self, name: str, alternative_value: Optional[Parameter] = None) -> Parameter:
        if alternative_value is None:
            alternative_value = Parameter(name, Parameter.Type.NOT_SET)

        res = self._parameters.get(name, alternative_value)
        if not isinstance(res, Parameter):
            res = Parameter(name, value=res)
        return res


    def loop_cb(self):
        
        # TODO : findo out what happened to is_shutdown
        # self._logger.info("jumping into while...")

        # while self.context.ok():
        #try:
        #    print("trying to catch some zzzs")
        #    # main_rate.sleep()
        #    # time.sleep(1)
        #except rclpy.exceptions.ROSInterruptException: # shutdown / stop ODrive??
        #    self._logger.error("interrupt exception")
        #    raise Exception("poop")
        #    #break
        
        # fast timer running, so do nothing and wait for any errors
        # print("seeing if timer active...")
        #if self.fast_timer_comms_active:
        #    print("comm is active")
        #    # continue
        
        # check for errors
        if self.driver:
            print("have driver")
            try:
                # driver connected, but fast_comms not active -> must be an error?
                # TODO: try resetting errors and recalibrating, not just a full disconnection
                error_string = self.driver.get_errors(clear=True)
                if error_string:
                    self._logger.error("Had errors, disconnecting and retrying connection.")
                    self._logger.error(error_string)
                    self.driver.disconnect()
                    self.status = "disconnected"
                    self.status_pub.publish(String(data=self.status))
                    self.driver = None
                else:
                    # must have called connect service from another node
                    self.fast_timer_comms_active = True
            except (ChannelBrokenException, ChannelDamagedException, AttributeError):
                self._logger.error("ODrive USB connection failure in main_loop.")
                self.status = "disconnected"
                self.status_pub.publish(String(data=self.status))
                self.driver = None
            except:
                self._logger.error("Unknown errors accessing ODrive:" + traceback.format_exc())
                self.status = "disconnected"
                self.status_pub.publish(String(data=self.status))
                self.driver = None
        
        if not self.driver:
            print("no driver")
            if self.connect_on_startup:
                self._logger.info("ODrive node started, but not connected.")
                if not self.connect_driver(std_srvs.srv.Trigger.Request(), std_srvs.srv.Trigger.Response())[0]:
                    self._logger.error("Failed to connect.") # TODO: can we check for timeout here?

        # TODO add back in when ready 
        # if self.publish_diagnostics:
        #    self.diagnostic_updater.setHardwareID(self.driver.get_version_string())
        

    def fast_timer_cb(self):

        time_now = self.get_clock().now()

        # in case of failure, assume some values are zero
        self.vel_l = 0
        self.vel_r = 0
        self.new_pos_l = 0
        self.new_pos_r = 0
        self.current_l = 0
        self.current_r = 0
        self.temp_v_l = 0.
        self.temp_v_r = 0.
        self.motor_state_l = "not connected" # undefined
        self.motor_state_r = "not connected"
        self.bus_voltage = 0.
        
        # Handle reading from Odrive and sending odometry
        if self.fast_timer_comms_active:
            # print("timer comms active")
            try:
                # check errors
                if self.driver is None:
                    self.fast_timer_comms_active = False
                    return
                error_string = self.driver.get_errors()
                if error_string:
                    self.fast_timer_comms_active = False
                else:
                    # reset watchdog
                    self.driver.feed_watchdog()
                    
                    # read all required values from ODrive for odometry
                    self.motor_state_l = self.driver.left_state()
                    self.motor_state_r = self.driver.right_state()
                    
                    self.encoder_cpr = self.driver.encoder_cpr
                    self.m_s_to_value = self.encoder_cpr/self.tyre_circumference # calculated

                    now_secs = time_now.seconds_nanoseconds()[0]
                    self.driver.update_time(now_secs)
                    self.vel_l = self.driver.left_vel_estimate()   # units: encoder counts/s
                    self.vel_r = -self.driver.right_vel_estimate() # neg is forward for right
                    self.new_pos_l = self.driver.left_pos()        # units: encoder counts
                    self.new_pos_r = -self.driver.right_pos()      # sign!
                    
                    # for temperatures
                    self.temp_v_l = self.driver.left_temperature()
                    self.temp_v_r = self.driver.right_temperature()
                    # for current
                    self.current_l = self.driver.left_current()
                    self.current_r = self.driver.right_current()
                    # voltage
                    self.bus_voltage = self.driver.bus_voltage()
                    
            except (ChannelBrokenException, ChannelDamagedException):
                self._logger.error("ODrive USB connection failure in fast_timer." + traceback.format_exc(1))
                self.fast_timer_comms_active = False
                self.status = "disconnected"
                self.status_pub.publish(String(data=self.status))
                self.driver = None
            except:
                self._logger.error("Fast timer ODrive failure:" + traceback.format_exc())
                self.fast_timer_comms_active = False
                
        # odometry is published regardless of ODrive connection or failure (but assumed zero for those)
        # as required by SLAM
        try:
            if self.publish_odom:
                self.pub_odometry()
            if self.publish_temperatures:
                self.pub_temperatures()
            if self.publish_current:
                self.pub_current()
            if self.publish_joint_angles:
                self.pub_joint_angles()

            # TODO add back in when ready
            #if self.publish_diagnostics:
            #    self.diagnostic_updater.update()
        except Exception as exc:
            self.get_logger().error(str(exc))
            self._logger.error("failed to pub")
        
        try:
            # check and stop motor if no vel command has been received in > 1s
            if self.fast_timer_comms_active:
                
                if self.driver:
                    secs_diff = time_now - self.last_cmd_vel_time
                    # convert duration with nanosecs to int
                    now_secs = (secs_diff.nanoseconds * 1e-9)
                    if now_secs > 0.5 and self.last_speed > 0:
                        self.driver.drive(0,0)
                        self.last_speed = 0
                        self.last_cmd_vel_time = time_now
                    # release motor after 10s stopped
                    if now_secs > 10.0 and self.driver.engaged:
                        print("auto disengage since not commands received")
                        self.driver.release() # and release
        except (ChannelBrokenException, ChannelDamagedException):
            self.get_logger().error("ODrive USB connection failure in cmd_vel timeout." + traceback.format_exc(1))
            self.fast_timer_comms_active = False
            self.driver = None
        except:
            self.get_logger().error("cmd_vel timeout unknown failure:" + traceback.format_exc())
            self.fast_timer_comms_active = False

        
        # handle sending drive commands.
        # from here, any errors return to get out
        if self.fast_timer_comms_active and not self.command_queue.empty():
            self._logger.info("have a command to exec")
            # check to see if we're initialised and engaged motor
            #try:
            #    if not self.driver.has_prerolled(): #ensure_prerolled():
            #        self._logger.warn("ODrive has not been prerolled, ignoring drive command.")
            #        motor_command = self.command_queue.get_nowait()
            #        return
            #except:
            #    self.get_logger().error("Fast timer exception on preroll." + traceback.format_exc())
            #    self.fast_timer_comms_active = False

            try:
                motor_command = self.command_queue.get_nowait()
            except queue.Empty:
                self.get_logger().error("Queue was empty??" + traceback.format_exc())
                return
            
            if motor_command[0] == 'drive':
                try:
                    self._logger.info("trying a drive command")
                    if self.publish_current and self.i2t_error_latch:
                        # have exceeded i2t bounds
                        return
                    
                    if not self.driver.engaged:
                        self.driver.engage()
                        self.status = "engaged"

                    print(motor_command)
                    left_linear_val, right_linear_val = motor_command[1]
                    self.driver.drive(left_linear_val, right_linear_val)
                    self.last_speed = max(abs(left_linear_val), abs(right_linear_val))
                    self.last_cmd_vel_time = time_now
                except (ChannelBrokenException, ChannelDamagedException):
                    self.get_logger().error("ODrive USB connection failure in drive_cmd." + traceback.format_exc(1))
                    self.fast_timer_comms_active = False
                    self.driver = None
                except:
                    self._logger.error("motor drive unknown failure: {0}".format(traceback.format_exc()))
                    self.fast_timer_comms_active = False
            
            elif motor_command[0] == 'release':
                pass


    def terminate(self):
        self._logger.info("attempting to terminate")
        try:
            if self.fast_timer:
                self.fast_timer.destroy()
            if self.driver:
                self.driver.release()
        except Exception as exc:
            self._logger.warn("failed to terminate")
            raise exc
    
    # ROS services
    def connect_driver(self, request: std_srvs.srv.Trigger.Request, resp:std_srvs.srv.Trigger.Response) -> std_srvs.srv.Trigger.Response:
        self._logger.info("attempting to connect driver")

        if self.driver:
            resp.success = False
            resp.message = "Already connected."
            self._logger.warn("already connected")
            return resp
        try:
            ODriveClass = ODriveInterfaceAPI if not self.sim_mode else ODriveInterfaceSimulator
            
            self.driver = ODriveClass(logger=ROSLogger(self))
            self._logger.info("Connecting to ODrive...")
            if not self.driver.connect(serial_number=self.serial_number, right_axis=self.axis_for_right):
                self.driver = None
                self._logger.error("Failed to connect.")
                resp.success = False
                resp.message = "Failed to connect."
                return resp
            
            self._logger.info("ODrive connected.")
            # okay, connected,
            self.m_s_to_value = self.driver.encoder_cpr/self.tyre_circumference
            
            if self.publish_odom:
                self.old_pos_l = self.driver.left_axis.encoder.pos_cpr
                self.old_pos_r = self.driver.right_axis.encoder.pos_cpr
            
            self.fast_timer_comms_active = True
            
            self.status = "connected"
            self.status_pub.publish(String(data=self.status))
        except Exception as exc:
            raise exc

        resp.success = True
        resp.message = "ODrive connected successfully."
        return resp
    
    def disconnect_driver(self, request: std_srvs.srv.Trigger.Request, resp:std_srvs.srv.Trigger.Response) -> std_srvs.srv.Trigger.Response:
        self._logger.info("attempting to disconnect driver")

        if self.driver is None:
            self._logger.error("Not connected.")
            resp.success = False
            resp.message = "Not connected."
            return resp
        
        try:
            did_disconnect = self.driver.disconnect()
            if not did_disconnect:
                resp.success = False
                resp.message = "Failed disconnection, but try reconnecting."
                return resp
        except:
            self._logger.error('Error while disconnecting: {}'.format(traceback.format_exc()))
        finally:
            self.status = "disconnected"
            self.status_pub.publish(String(data=self.status))
            self.driver = None
        
        resp.success = True
        resp.message = "Disconnection success."
        return resp
    
    def calibrate_motor(self, request: std_srvs.srv.Trigger.Request, resp:std_srvs.srv.Trigger.Response) -> std_srvs.srv.Trigger.Response:
        self._logger.info("attempting to calibrate motor")

        if not self.driver:
            self._logger.error("Not conected.")
            resp.success = False
            resp.message = "Not connected."
        
        try:
            if self.has_preroll:
                self.odometry_update_enabled = False # disable odometry updates while we preroll
                if not self.driver.preroll(wait=True):
                    self.status = "preroll_fail"
                    self.status_pub.publish(String(data=self.status))
                    resp.success = True
                    resp.message = "Failed preroll."
                    return resp
                
                self.status_pub.publish(String(data="ready"))
                time.sleep(1)
                self.odometry_update_enabled = True
            else:
                if not self.driver.calibrate():
                    resp.success = False
                    resp.message = "Failed calibration."
                    return resp
        except Exception as exc:
            raise exc
        
        resp.success = True
        resp.message = "Calibration success."
        return resp
    
    def engage_motor(self, request: std_srvs.srv.Trigger.Request, resp:std_srvs.srv.Trigger.Response) -> std_srvs.srv.Trigger.Response:
        if not self.driver:
            self.get_logger().error("Not connected.")
            resp.success = False
            resp.message = "Not connected."
            return resp
        #if not self.driver.has_prerolled():
        #    resp.success = False
        #    resp.message = "Not prerolled."
        #    return resp
        if not self.driver.engage():
            resp.success = False
            resp.message = "Failed to engage motor."
            return resp

        resp.success = True
        resp.message = "Engage motor success."
        return resp
    
    def release_motor(self, request: std_srvs.srv.Trigger.Request, resp:std_srvs.srv.Trigger.Response) -> std_srvs.srv.Trigger.Response:
        if not self.driver:
            self.get_logger().error("Not connected.")
            resp.success = False
            resp.message = "Not connected."
            return resp
        if not self.driver.release():
            resp.success = False
            resp.message = "Failed to release motor."
            return resp

        resp.success = True
        resp.message = "Release motor success."
        return resp
        
    def enable_odometry_update_service(self, request: std_srvs.srv.Trigger.Request, resp:std_srvs.srv.Trigger.Response) -> std_srvs.srv.Trigger.Response:
        enable = request.data
        
        if enable:
            self.odometry_update_enabled = True
            resp.success = True
            resp.message = "Odometry enabled."
        else:
            self.odometry_update_enabled = False
            resp.success = True
            resp.message = "Odometry disabled."
        return resp
    
    def reset_odometry(self, request: std_srvs.srv.Trigger.Request, resp:std_srvs.srv.Trigger.Response) -> std_srvs.srv.Trigger.Response:
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        resp.success = True
        resp.message = "Odometry reset."
        return resp
    
    # Helpers and callbacks
    
    def convert(self, forward, ccw):
        angular_to_linear = ccw * (self.wheel_track/2.0) 
        left_linear_val  = int((forward - angular_to_linear) * self.m_s_to_value)
        right_linear_val = int((forward + angular_to_linear) * self.m_s_to_value)
    
        return left_linear_val, right_linear_val

    def cmd_vel_callback(self, msg):
        self._logger.info("Received a /cmd_vel message!")
        self._logger.info("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        self._logger.info("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

        # rostopic pub -r 1 /commands/motor/current std_msgs/Float64 -- -1.0

        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities into motor commands
        
        # 3600 ERPM = 360 RPM ~= 6 km/hr
        
        #angular_to_linear = msg.angular.z * (wheel_track/2.0) 
        #left_linear_rpm  = (msg.linear.x - angular_to_linear) * m_s_to_erpm
        #right_linear_rpm = (msg.linear.x + angular_to_linear) * m_s_to_erpm
        left_linear_val, right_linear_val = self.convert(msg.linear.x, msg.angular.z)
        
        # if wheel speed = 0, stop publishing after sending 0 once. #TODO add error term, work out why VESC turns on for 0 rpm
        
        # Then set your wheel speeds (using wheel_left and wheel_right as examples)
        #self.left_motor_pub.publish(left_linear_rpm)
        #self.right_motor_pub.publish(right_linear_rpm)
        #wheel_left.set_speed(v_l)
        #wheel_right.set_speed(v_r)
        
        #self.get_logger().debug("Driving left: %d, right: %d, from linear.x %.2f and angular.z %.2f" % (left_linear_val, right_linear_val, msg.linear.x, msg.angular.z))
        try:
            drive_command = ('drive', (left_linear_val, right_linear_val))
            self.command_queue.put_nowait(drive_command)
        except queue.Full:
            pass
            
        self.last_cmd_vel_time = self._clock.now()
        
    def pub_diagnostics(self, stat):
        stat.add("Status", self.status)
        stat.add("Motor state L", self.motor_state_l) 
        stat.add("Motor state R", self.motor_state_r)
        stat.add("FET temp L (C)", round(self.temp_v_l,1))
        stat.add("FET temp R (C)", round(self.temp_v_r,1))
        stat.add("Motor temp L (C)", "unimplemented")
        stat.add("Motor temp R (C)", "unimplemented")
        stat.add("Motor current L (A)", round(self.current_l,1))
        stat.add("Motor current R (A)", round(self.current_r,1))
        stat.add("Voltage (V)", round(self.bus_voltage,2))
        stat.add("Motor i2t L", round(self.left_energy_acc,1))
        stat.add("Motor i2t R", round(self.right_energy_acc,1))
        
        # https://github.com/ros/common_msgs/blob/jade-devel/diagnostic_msgs/msg/DiagnosticStatus.msg
        # TODO add this back when eloquent has it...
        if self.status == "disconnected":
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Not connected")
        else:
            if self.i2t_error_latch:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "i2t overheated, drive ignored until cool")
            elif self.left_energy_acc > self.i2t_warning_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Left motor over i2t warning threshold")
            elif self.left_energy_acc > self.i2t_error_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Left motor over i2t error threshold")
            elif self.right_energy_acc > self.i2t_warning_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Right motor over i2t warning threshold")
            elif self.right_energy_acc > self.i2t_error_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Right motor over i2t error threshold")
            # Everything is okay:
            else:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Running")
        
        
    def pub_temperatures(self):
        # https://discourse.odriverobotics.com/t/odrive-mosfet-temperature-rise-measurements-using-the-onboard-thermistor/972
        # https://discourse.odriverobotics.com/t/thermistors-on-the-odrive/813/7
        # https://www.digikey.com/product-detail/en/murata-electronics-north-america/NCP15XH103F03RC/490-4801-1-ND/1644682
        #p3 =  363.0
        #p2 = -459.2
        #p1 =  308.3
        #p0 =  -28.1
        #
        #vl = self.temp_v_l
        #vr = self.temp_v_r

        #temperature_l = p3*vl**3 + p2*vl**2 + p1*vl + p0
        #temperature_r = p3*vr**3 + p2*vr**2 + p1*vr + p0
        
        #print(temperature_l, temperature_r)
        
        self.temperature_publisher_left.publish(self.temp_v_l)
        self.temperature_publisher_right.publish(self.temp_v_r)
        
    # Current publishing and i2t calculation
    i2t_current_nominal = 2.0
    i2t_update_rate = 0.01
    
    def pub_current(self):
        self.current_publisher_left.publish(float(self.current_l))
        self.current_publisher_right.publish(float(self.current_r))
        
        now = time.time()
        
        if not hasattr(self, 'last_pub_current_time'):
            self.last_pub_current_time = now
            self.left_energy_acc = 0
            self.right_energy_acc = 0
            return
            
        # calculate and publish i2t
        dt = now - self.last_pub_current_time
        
        power = max(0, self.current_l**2 - self.i2t_current_nominal**2)
        energy = power * dt
        self.left_energy_acc *= 1 - self.i2t_update_rate * dt
        self.left_energy_acc += energy
        
        power = max(0, self.current_r**2 - self.i2t_current_nominal**2)
        energy = power * dt
        self.right_energy_acc *= 1 - self.i2t_update_rate * dt
        self.right_energy_acc += energy
        
        self.last_pub_current_time = now
        
        self.i2t_publisher_left.publish(float(self.left_energy_acc))
        self.i2t_publisher_right.publish(float(self.right_energy_acc))
        
        # stop odrive if overheated
        if self.left_energy_acc > self.i2t_error_threshold or self.right_energy_acc > self.i2t_error_threshold:
            if not self.i2t_error_latch:
                self.driver.release()
                self.status = "overheated"
                self.i2t_error_latch = True
                self.get_logger().error("ODrive has exceeded i2t error threshold, ignoring drive commands. Waiting to cool down.")
        elif self.i2t_error_latch:
            if self.left_energy_acc < self.i2t_resume_threshold and self.right_energy_acc < self.i2t_resume_threshold:
                # have cooled enough now
                self.status = "ready"
                self.i2t_error_latch = False
                self.get_logger().error("ODrive has cooled below i2t resume threshold, ignoring drive commands. Waiting to cool down.")
        
        
    #     current_quantizer = 5
    #
    #     self.left_current_accumulator += self.current_l
    #     self.right_current_accumulator += self.current_r
    #
    #     self.current_loop_count += 1
    #     if self.current_loop_count >= current_quantizer:
    #         self.current_publisher_left.publish(float(self.left_current_accumulator) / current_quantizer)
    #         self.current_publisher_right.publish(float(self.right_current_accumulator) / current_quantizer)
    #
    #         self.current_loop_count = 0
    #         self.left_current_accumulator = 0.0
    #         self.right_current_accumulator = 0.0

    def pub_odometry(self):
        time_now = self.get_clock().now()
        self.odom_msg.header.stamp = time_now.to_msg()
        self.tf_msg.header.stamp = time_now.to_msg()
        
        wheel_track = self.wheel_track   # check these. Values in m
        tyre_circumference = self.tyre_circumference
        # self.m_s_to_value = encoder_cpr/tyre_circumference set earlier
        
        # if odometry updates disabled, just return the old position and zero twist.
        if not self.odometry_update_enabled:
            self.odom_msg.twist.twist.linear.x = 0.
            self.odom_msg.twist.twist.angular.z = 0.
            
            # but update the old encoder positions, so when we restart updates
            # it will start by giving zero change from the old position.
            self.old_pos_l = self.new_pos_l
            self.old_pos_r = self.new_pos_r
            
            self.odom_publisher.publish(self.odom_msg)
            if self.publish_tf:
                self.tf_publisher.sendTransform(self.tf_msg)
            
            return
        
        # Twist/velocity: calculated from motor values only
        s = tyre_circumference * (self.vel_l+self.vel_r) / (2.0*self.encoder_cpr)
        w = tyre_circumference * (self.vel_r-self.vel_l) / (wheel_track * self.encoder_cpr) # angle: vel_r*tyre_radius - vel_l*tyre_radius
        self.odom_msg.twist.twist.linear.x = s
        self.odom_msg.twist.twist.angular.z = w
    
        #self.get_logger().info("vel_l: % 2.2f  vel_r: % 2.2f  vel_l: % 2.2f  vel_r: % 2.2f  x: % 2.2f  th: % 2.2f  pos_l: % 5.1f pos_r: % 5.1f " % (
        #                vel_l, -vel_r,
        #                vel_l/encoder_cpr, vel_r/encoder_cpr, self.odom_msg.twist.twist.linear.x, self.odom_msg.twist.twist.angular.z,
        #                self.driver.left_axis.encoder.pos_cpr, self.driver.right_axis.encoder.pos_cpr))
        
        # Position
        delta_pos_l = self.new_pos_l - self.old_pos_l
        delta_pos_r = self.new_pos_r - self.old_pos_r
        
        self.old_pos_l = self.new_pos_l
        self.old_pos_r = self.new_pos_r
        
        # Check for overflow. Assume we can't move more than half a circumference in a single timestep. 
        half_cpr = self.encoder_cpr/2.0
        if   delta_pos_l >  half_cpr: delta_pos_l = delta_pos_l - self.encoder_cpr
        elif delta_pos_l < -half_cpr: delta_pos_l = delta_pos_l + self.encoder_cpr
        if   delta_pos_r >  half_cpr: delta_pos_r = delta_pos_r - self.encoder_cpr
        elif delta_pos_r < -half_cpr: delta_pos_r = delta_pos_r + self.encoder_cpr
        
        # counts to metres
        delta_pos_l_m = delta_pos_l / self.m_s_to_value
        delta_pos_r_m = delta_pos_r / self.m_s_to_value
    
        # Distance travelled
        d = (delta_pos_l_m+delta_pos_r_m)/2.0  # delta_ps
        th = (delta_pos_r_m-delta_pos_l_m)/wheel_track # works for small angles
    
        xd = math.cos(th)*d
        yd = -math.sin(th)*d
    
        # elapsed time = event.last_real, event.current_real
        #elapsed = (event.current_real-event.last_real).to_sec()
        # calc_vel: d/elapsed, th/elapsed
    
        # Pose: updated from previous pose + position delta
        self.x += math.cos(self.theta)*xd - math.sin(self.theta)*yd
        self.y += math.sin(self.theta)*xd + math.cos(self.theta)*yd
        self.theta = (self.theta + th) % (2*math.pi)
    
        #self.get_logger().info("dl_m: % 2.2f  dr_m: % 2.2f  d: % 2.2f  th: % 2.2f  xd: % 2.2f  yd: % 2.2f  x: % 5.1f y: % 5.1f  th: % 5.1f" % (
        #                delta_pos_l_m, delta_pos_r_m,
        #                d, th, xd, yd,
        #                self.x, self.y, self.theta
        #                ))
    
        # fill odom message and publish
        
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        q = transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        self.odom_msg.pose.pose.orientation.z = q[2] # math.sin(self.theta)/2
        self.odom_msg.pose.pose.orientation.w = q[3] # math.cos(self.theta)/2
    
        #self.get_logger().info("theta: % 2.2f  z_m: % 2.2f  w_m: % 2.2f  q[2]: % 2.2f  q[3]: % 2.2f (q[0]: %2.2f  q[1]: %2.2f)" % (
        #                        self.theta,
        #                        math.sin(self.theta)/2, math.cos(self.theta)/2,
        #                        q[2],q[3],q[0],q[1]
        #                        ))
    
        #self.odom_msg.pose.covariance
         # x y z
         # x y z
    
        self.tf_msg.transform.translation.x = self.x
        self.tf_msg.transform.translation.y = self.y
        #self.tf_msg.transform.rotation.x
        #self.tf_msg.transform.rotation.x
        self.tf_msg.transform.rotation.z = q[2]
        self.tf_msg.transform.rotation.w = q[3]
        
        if self.publish_raw_odom:
            self.raw_odom_publisher_encoder_left.publish(self.new_pos_l)
            self.raw_odom_publisher_encoder_right.publish(self.new_pos_r)
            self.raw_odom_publisher_vel_left.publish(self.vel_l)
            self.raw_odom_publisher_vel_right.publish(self.vel_r)
        
        # ... and publish!
        self.odom_publisher.publish(self.odom_msg)
        if self.publish_tf:
            self.tf_publisher.sendTransform(self.tf_msg)            
    
    def pub_joint_angles(self):
        time_now = self.get_clock().now()
        jsm = self.joint_state_msg
        jsm.header.stamp = time_now.to_msg()
        if self.driver:
            jsm.position[0] = 2*math.pi * self.new_pos_l  / self.encoder_cpr
            jsm.position[1] = 2*math.pi * self.new_pos_r / self.encoder_cpr
            
        self.joint_state_publisher.publish(jsm)


def main(args=None):
    rclpy.init(args=args)
    print(args)

    odn = ODriveNode()
    try:
        #rclpy.spin(odn)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(odn)
        executor.spin()
    except KeyboardInterrupt:
        print("exiting")
    finally:
        executor.shutdown()
        odn.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

