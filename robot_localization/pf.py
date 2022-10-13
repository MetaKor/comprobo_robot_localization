#!/usr/bin/env python3

""" CompRobo Particle Filter
    Authors: Luke Raus & Han Vakil
    Fall 2022
"""

import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from rclpy.duration import Duration
from occupancy_field import OccupancyField
from helper_functions import TFHelper
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler
import math
import time
import numpy as np

class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of KeyboardInterruptthe hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """ 
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0.0),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

    # TODO: define additional helper functions if needed
    def drive(self, distance):
        """
        Translate the particle forwards along its current heading
        args:
            distance: float, m, how far to translate
        """
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)
    
    def turn(self, angle):
        """
        Turn the particle
        args:
            angle: float, rad, how far to rotate (pos forr ccw like standard)
        """
        self.theta += angle
    
    def point_at_distance(self, distance):
        """
        Returns the point distance away along the direction of the particle
        args:
            distance: float, m, how far ahead to go to get the point
        returns:
            point: tuple (float, float), (m, m): (x,y) the coordinates of the point
        """
        x_coord = self.x + distance * math.cos(self.theta)
        y_coord = self.y + distance * math.sin(self.theta)
        point = (x_coord, y_coord)
        return point

    def point_at_dist_deg(self, distance, ang):
        """
        Returns the point distance away along the direction of the particle
        args:
            distance: float, m, how far ahead to go to get the point
            ang: float, deg, at what angle (in robot frame) to get the point
        returns:
            point: tuple (float, float), (m, m): (x,y) the coordinates of the point
        """
        ang_global = self.theta + math.radians(ang)
        x_coord = self.x + distance * math.cos(ang_global)
        y_coord = self.y + distance * math.sin(ang_global)
        point = (x_coord, y_coord)
        return point
    


class ParticleFilter(Node):
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            last_scan_timestamp: this is used to keep track of the clock when using bags
            scan_to_process: the scan that our run_loop should process next
            occupancy_field: this helper class allows you to query the map for distance to closest obstacle
            transform_helper: this helps with various transform operations (abstracting away the tf2 module)
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            thread: this thread runs your main loop
    """
    def __init__(self):
        super().__init__('pf')
        self.base_frame = "base_footprint"   # name of the robot base frame
        self.map_frame = "map"               # name of the map coordinate frame
        self.odom_frame = "odom"             # name of the odometry coordinate frame
        self.scan_topic = "scan"             # topic where we will get laser scans from 

        self.n_particles = 1           # the number of particles to use

        self.d_thresh = 0.1             # [m] amount of linear movement before performing an update
        self.a_thresh = math.pi/10      # [rad] amount of angular movement before performing an update

        self.init_xy_dev = 0.01  # 0.2          # [m] standard deviation of x & y position of particles in initial cloud
        self.init_theta_dev = 0.01  # 0.3     # [rad] standard deviation of orientation of particles in initial cloud

        # TODO: define additional constants if needed

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.update_initial_pose, 10)

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(PoseArray, "particlecloud", qos_profile_sensor_data)

        # laser_subscriber listens for data from the lidar
        self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        # your particle cloud will go here
        self.particle_cloud = []

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)

    def pub_latest_transform(self):
        """ This function takes care of sending out the map to odom transform """
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(seconds=0.1)
        self.transform_helper.send_last_map_to_odom_transform(self.map_frame, self.odom_frame, postdated_timestamp)

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        """ This is the main run_loop of our particle filter.  It checks to see if
            any scans are ready and to be processed and will call several helper
            functions to complete the processing.
            
            You do not need to modify this function, but it is helpful to understand it.
        """
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(self.odom_frame,
                                                                           self.base_frame,
                                                                           msg.header.stamp)
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0):
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return
        
        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(msg, self.base_frame)
        #print("r[0]={0}, theta[0]={1}".format(r[0], theta[0]))
        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        #print("x: {0}, y: {1}, yaw: {2}".format(*new_odom_xy_theta))

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particle_cloud:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            print(f"new_odom_xy_theta:")
            # we have moved far enough to do an update!
            self.update_particles_with_odom()            # update particle location based on odometry
            self.update_particles_with_laser(r, theta)   # update particle likelihood  based on laser scan

            for particle in self.particle_cloud:
                print(f"x: {particle.x}   y: {particle.y}   theta: {particle.theta}   weight: {particle.w}")
            """
            self.update_robot_pose()                # update robot's pose based on particles
            self.resample_particles()               # resample particles to focus on areas of high density
            """
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh


    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            There are two logical methods for this:
                (1): compute the mean pose
                (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        #starting with the mean pose because that's easiest
        guessed_x = 0
        guessed_y = 0
        guessed_theta = 0

        for particle in self.particle_cloud:
            guessed_x += particle.x * particle.w
            guessed_y += particle.y * particle.w
            guessed_theta += particle.theta * particle.w

        print(f"guessed x: {guessed_x} guessed y: {guessed_y} guessed theta: {guessed_theta}")

        self.robot_pose = Pose()

        self.robot_pose.position.x = guessed_x
        self.robot_pose.position.y = guessed_y
        self.robot_pose.orientation.z = guessed_theta

        # the guessed x, y and theta match the particle cloud just fine (map frame x, y theta not odom). 
        # Based on the fix map to odom helper it seems to want map frame for this. However the robot
        # model isn't placed at these coords in the map frame, it seems (but I'm not sure) that it's placed
        # at these coords in the odom frame and I'm not sure why. Some previous thoughts/why I think this
        # are below: 

        # NOTE FROM HAN: not sure this is assigning where I think I'm assigning it.
        # guessed_x and guessed_y can be small (-0.5 and -1.8) but the robot model ends 
        # up way out of the gauntlet. It might be something to do with the frames the
        # transform is based off? If odom is moving or something? I think it shouls all
        # be based off the map so this shouldn't happen but not sure.

        # having the neato spin in a circle in place casuses the guessed position to jump
        # in a cycle, it's in place when theta=0 and most off when theta=pi
        # i'm guessing this means the issue is with frames, maybe i'm interpreting the
        # frames wrong and using the wrong ones? the below lines are provided though so that
        # should probably be right

        # jumps very little when at 0,0 in global, a lot when far from 0, 0

        # when initial pose going right, driving forwards results in arrow going up (trig error?)


        self.transform_helper.fix_map_to_odom_transform(self.robot_pose,
                                                        self.odom_pose)


    def update_particles_with_odom(self):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        
        # Compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        # For motion model, see Thrun et al, pg 133-139
        # Each motion update consists of:
        #    an initial pure turn of (ang1) [rads],
        #    a straight-ahead drive of length (dist) [m],
        #    and finally another pure turn of (ang2) [rads] 
        ang1 = math.atan2(delta[1], delta[0]) - self.current_odom_xy_theta[2]
        dist = math.sqrt(delta[0] ** 2 + delta[1] ** 2)
        ang2 = delta[2] - ang1

        # TODO Fix potential angle wrap issues after adding noise

        # Noise parameters & use below comes from Thrun et al, pg 136
        # They are robot-specific and specify noise in motor model...
        #    However, Thrun doesn't develop much intuition for what each means
        #    but you can get a good sense by loooking at units
        #    I have set them all to (empirically) reasonable values
        a1 = 0.08     # [noise_rad/rad]
        a2 = 0.1      # [noise_rad/m]
        a3 = 0.08     # [noise_m/m]
        a4 = 0.00001  # [noise_m/rad]

        for particle in self.particle_cloud:
            #print(f"old x: {particle.x} old y: {particle.y} old theta: {particle.theta}")
            
            # Noisy update equations from Thrun et al, pg 136
            ang1_noisy  = ang1 + np.random.normal( scale=( a1*abs(ang1) + a2*dist ) )
            dist_noisy  = dist + np.random.normal( scale=( a3*dist + a4*(abs(ang1)+abs(ang2)) ) )
            ang2_noisy  = ang2 + np.random.normal( scale=( a1*abs(ang2) + a2*dist ) )
            
            print(f"ang1: {ang1} dist: {dist} ang2: {ang2}")
            print(f"ang1_noisy: {ang1_noisy} dist_noisy: {dist_noisy} ang2_noisy: {ang2_noisy}")
            #print(f"delta: {delta}")
            particle.turn(ang1_noisy)
            particle.drive(dist_noisy)
            particle.turn(ang2_noisy)
            #print(f"new x: {particle.x} new y: {particle.y} new theta: {particle.theta}")
            

    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample in helper_functions.py.
        """
        # make sure the distribution is normalized
        self.normalize_particles()
        # TODO: fill out the rest of the implementation


    def update_particles_with_laser(self, r, theta):
        """ Updates the particle weights in response to the scan data
            r: the distance readings to obstacles
            theta: the angle relative to the robot frame for each corresponding reading 
        """
        error_hit = 0.1          # [m] The maximum unpenalized distance error

        scan_resolution = 20     # Process 1 scan out of this many scans

        for particle in self.particle_cloud:
            particle_likelihood = 1.0;

            for num_scan in range(0, len(theta), scan_resolution):
                # Get distance and angle from actual measured scan
                scan_ang = theta[ int(num_scan) ]
                scan_dist = r[ int(num_scan) ]

                # Project scan in particle frame and find dist to nearest obstacle
                (proj_x, proj_y) = particle.point_at_dist_deg(scan_dist, scan_ang)
                min_obstacle_dist = OccupancyField.get_closest_obstacle_distance(self.occupancy_field, proj_x, proj_y)
                error = min_obstacle_dist

                if math.isnan(error):
                    # Penalize NaN readings with constant likelihood multiplier
                    scan_likelihood = 0.05
                else:
                    # Piecewise approximation for gaussian:
                    #    If error less than error_hit, likelihood is 1
                    #    If error larger than error_hit, error is proportional to 1/(error^3), but we
                    #    must shift the function by (1-error_hit) so it's exactly 1 (and not larger) at error_hit
                    scan_likelihood = (error <= error_hit) + (error > error_hit) * ((error+(1.0-error_hit))**(-3))

                print(f"scan angle: {scan_ang}    scan_dist: {scan_dist}   error: {error}   scan_like: {scan_likelihood}")


                particle_likelihood *= scan_likelihood

            particle.w = particle_likelihood

        #self.normalize_particles()


    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        print("updating initial pose")
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud_uniform(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used """  
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        initial_x = xy_theta[0]
        initial_y = xy_theta[1]
        initial_theta = xy_theta[2]
        print(f"Initial x: {initial_x}. Initial y: {initial_y}. Initial theta: {initial_theta}")
        self.particle_cloud = []
        size_per_axis = math.floor(self.n_particles ** (1/3)) #distribute particles in 3 axes (x y theta) around initial estimate
        print(size_per_axis)
        initial_linear_range = 0.5 #distance to distribute those initial particles across, m, total (not this in either dir)
        initial_theta_range = 3.14159 / 2 #angle to distribute initial particles across, rad, total (not this in either dir)
        for i in range(size_per_axis):
            for j in range(size_per_axis):
                for k in range(size_per_axis):
                    x_val = -1/2 * initial_linear_range + initial_linear_range / size_per_axis * (i+0.5) + initial_x
                    y_val = -1/2 * initial_linear_range + initial_linear_range / size_per_axis * (j+0.5) + initial_y
                    z_val = -1/2 * initial_theta_range + initial_theta_range / size_per_axis * (k+0.5) + initial_theta
                    self.particle_cloud.append(Particle(x_val,y_val,z_val,1/self.n_particles))
        # should we replace the initialization with a gaussian? probably

        self.normalize_particles()

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud by sampling from a normal distribution.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used """  
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        initial_x = xy_theta[0]
        initial_y = xy_theta[1]
        initial_theta = xy_theta[2] 
        print(f"Initial x: {initial_x}. Initial y: {initial_y}. Initial theta: {initial_theta}")
        self.particle_cloud = []

        # TODO: Draw (1 x n_particles) np arrays with sample values from normal distribution; 1 for x, y, theta
        for i in range(self.n_particles):
            x     = initial_x + np.random.normal(scale=self.init_xy_dev, size=None)
            y     = initial_y + np.random.normal(scale=self.init_xy_dev, size=None)
            theta = initial_theta + np.random.normal(scale=self.init_theta_dev, size=None)
            self.particle_cloud.append(Particle(x, y, theta, 1.0))

        # Normalizing takes care of initial particle weights of 1
        self.normalize_particles()

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        total_particle_weight = 0
        final_total_particle_weight = 0
        for particle in self.particle_cloud:
            total_particle_weight += particle.w
        for particle in self.particle_cloud:
            particle.w = particle.w / total_particle_weight
            final_total_particle_weight += particle.w
        print(f"Total particle weight initial: {total_particle_weight}")
        print(f"Total particle weight final: {final_total_particle_weight}")



    def publish_particles(self, timestamp):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(PoseArray(header=Header(stamp=timestamp,
                                            frame_id=self.map_frame),
                                  poses=particles_conv))


    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop 
        if self.scan_to_process is None:
            self.scan_to_process = msg

def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
