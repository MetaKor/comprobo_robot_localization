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
from helper_functions import TFHelper, draw_integer_sample
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler
import numpy as np
import time


class ParticleCloud(object):
    """ Represents a set of hypotheses (particles) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, n=500):
        """ Construct a new ParticleCloud
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight
            n: the number of particles """ 
        self.N     = n
        self.X     = np.zeros(self.N)
        self.Y     = np.zeros(self.N)
        self.Theta = np.zeros(self.N)
        # Set each weight to 1/n so weights sum to 1
        self.W = np.ones(self.N)
        self.normalize()


    def normalize(self):
        """ Normalizes weights so that they sum to 1.0 and form a valid distribution """
        self.W /= np.sum(self.W)


    def initialize_from_hypothesis(self, x, y, theta, x_dev, y_dev, theta_dev):
        """ Initialize cloud around guess of (x, y, theta) as Gaussian with
            standard deviations on each attribute of (x_dev, y_dev, theta_dev)
        """
        self.X     =     x + np.random.normal(scale=x_dev,     size=self.N)
        self.Y     =     y + np.random.normal(scale=y_dev,     size=self.N)
        self.Theta = theta + np.random.normal(scale=theta_dev, size=self.N)
        self.W     = np.ones(self.N)
        self.normalize()


    def get_mode_particle(self):
        """ Get mode particle, whose x,y,theta are each expected value of cloud's x,y,theta
            returns:
                mode_particle: 3-float tuple, representing x,y,theta
        """
        self.normalize()
        mode_x     = np.sum(self.W * self.X)
        mode_y     = np.sum(self.W * self.Y)
        mode_theta = np.sum(self.W * self.Theta)

        mode_particle = (mode_x, mode_y, mode_theta)
        return mode_particle


    def get_particle_from_index(self, index):
        """ Return the x,y,theta,w values of the (index)-th particle in cloud
        """
        return ( self.X[index], self.Y[index], self.Theta[index], self.W[index] )


    def unwrap_angles(self):
        self.Theta = ( self.Theta + np.pi ) % (2 * np.pi ) - np.pi


    def drive_each(self, distances):
        """ Translate each particle forwards along current headings
            args:
                distances: 1xN float array, m, how far to translate
        """
        self.X += distances * np.cos(self.Theta)
        self.Y += distances * np.sin(self.Theta)


    def turn_each(self, angles):
        """
        Turn each particle by corresponding angle
        args:
            angle: 1xN float array, rad, how far to rotate (pos forr ccw like standard)
        """
        self.Theta += angles
        self.unwrap_angles()


    def points_at_dist_deg(self, distance, angle):
        """
        Returns the points that are (distances) away at (angle) in the frame of each particle
        args:
            distance: float, m, how far ahead to go to get each point
            angle: float, deg, at what angle (in robot frames) to get each point
        returns:
            (X_coords, Y_coords): tuple (1xN array, 1xN array), (m, m), the coordinates of the points
        """
        angles_global = self.Theta + np.radians(angle)
        X_coords = self.X + distance * np.cos(angles_global)
        Y_coords = self.Y + distance * np.sin(angles_global)
        return (X_coords, Y_coords)


    def as_poses(self):
        """ Convert each particle to a geometry_msgs/Pose message """
        poses = []
        for index in range(self.N):
            q = quaternion_from_euler(0, 0, self.Theta[index])
            pose = Pose(position=Point(x=self.X[index], y=self.Y[index], z=0.0),
                        orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
            poses.append(pose)
        return poses





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
        self.map_frame  = "map"              # name of the map coordinate frame
        self.odom_frame = "odom"             # name of the odometry coordinate frame
        self.scan_topic = "scan"             # topic where we will get laser scans from 

        self.n_particles = 10            # the number of particles to use

        self.d_thresh = 0.1              # [m] amount of linear movement before performing an update
        self.a_thresh = np.pi/8          # [rad] amount of angular movement before performing an update

        self.init_x_dev = 0.3            # [m] standard deviation of x position of particles in initial cloud
        self.init_y_dev = 0.3            # [m] standard deviation of x position of particles in initial cloud
        self.init_theta_dev = 0.5        # [rad] standard deviation of orientation of particles in initial cloud

        self.scan_error_threshold = 0.1  # [m] The maximum unpenalized distance error
        self.scan_resolution = 1         # Process one scan per every this many scans


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

        # create particle cloud object
        self.particles = ParticleCloud(n=self.n_particles)

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

        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particles:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):

            # we have moved far enough to do an update!
            self.update_particle_locations_with_odom()           # update particle location based on odometry
            self.update_particle_weights_with_laser(r, theta)    # update particle likelihood  based on laser scan
            self.update_robot_pose_estimate()                    # update robot's pose based on particles
            self.resample_particles()                            # resample particles to focus on areas of high density

        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return np.abs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or \
               np.abs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or \
               np.abs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh


    def update_particle_locations_with_odom(self):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        
        # Compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])
            prev_odom_xy_theta = self.current_odom_xy_theta

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        # For motion model, see Thrun et al, pg 133-139
        # Each motion update consists of:
        #    an initial pure turn of (ang1) [rads],
        #    a straight-ahead drive of length (dist) [m],
        #    and finally another pure turn of (ang2) [rads] 
        ang1 = np.arctan2(delta[1], delta[0]) - prev_odom_xy_theta[2]
        dist = np.sqrt( (delta[0] ** 2) + (delta[1] ** 2) )
        ang2 = delta[2] - ang1

        # Noise parameters & use below comes from Thrun et al, pg 136
        # They are robot-specific and specify noise in motor model...
        #    However, Thrun doesn't develop much intuition for what each means
        #    but you can get a good sense by loooking at units
        #    I have set them all to (empirically) reasonable values
        a1 = 0.04     # [noise_rad/rad]
        a2 = 0.08     # [noise_rad/m]
        a3 = 0.08     # [noise_m/m]
        a4 = 0.00001  # [noise_m/rad]

        # Noisy update equations from Thrun et al, pg 136
        # This creates numpy arrays with a noisy angle/dist for each particle
        Ang1_noisy  = ang1 + np.random.normal( scale=( a1*abs(ang1) + a2*dist ), 
                                                 size=self.n_particles )
        Dist_noisy  = dist + np.random.normal( scale=( a3*dist + a4*(abs(ang1)+abs(ang2)) ),
                                                 size=self.n_particles )
        Ang2_noisy  = ang2 + np.random.normal( scale=( a1*abs(ang2) + a2*dist ),
                                                 size=self.n_particles )

        # Ensure that noisy angles are between -pi & pi
        Ang1_noisy = self.transform_helper.angle_normalize( Ang1_noisy )
        Ang2_noisy = self.transform_helper.angle_normalize( Ang2_noisy )

        self.particles.turn_each( Ang1_noisy )
        self.particles.drive_each( Dist_noisy )
        self.particles.turn_each( Ang2_noisy )


    def update_particle_weights_with_laser(self, r, theta):
        """ Updates the particle weights in response to the scan data
            r: the distance readings to obstacles
            theta: the angle relative to the robot frame for each corresponding reading 
        """

        particle_likelihoods = np.ones(self.n_particles)

        thresh = self.scan_error_threshold

        for num_scan in range(0, len(theta), self.scan_resolution):
            # Get distance and angle from actual measured scan
            scan_ang = theta[ int(num_scan) ]
            scan_dist = r[ int(num_scan) ]

            # Project scan in particle frame and find dist to nearest obstacle
            (proj_X, proj_Y) = self.particles.points_at_dist_deg(scan_dist, scan_ang)
            min_obstacle_dists = OccupancyField.get_closest_obstacle_distance(self.occupancy_field, proj_X, proj_Y)
            errors = min_obstacle_dists

            # Replace any NaNs with a constant distance replacement of our choosing
            errors = np.nan_to_num(errors, nan=2.0)

            # The likelihood of this scan per particle is piecewise approxmimated as a Gaussian as follows:
            #    > 1.0 if the error was within the threshold
            #    > Proportional to 1/(error^3) if error beyond threshold, but we must must shift
            #        the input by (1-thresh so output is exactly 1.0 when error=thresh
            scan_likelihoods = (errors <= thresh) * 1.0 \
                             + (errors > thresh)  * ( (errors + (1.0-thresh)) ** (-3) )

            particle_likelihoods *= scan_likelihoods

        self.particles.W = particle_likelihoods
        self.particles.normalize()


    def update_robot_pose_estimate(self):
            """ Update the estimate of the robot's pose given the updated particles.
                There are two logical methods for this:
                    (1): compute the mean pose
                    (2): compute the most likely pose (i.e. the mode of the distribution)
            """
            # Take mode particle of cloud
            [guessed_x, guessed_y, guessed_theta] = self.particles.get_mode_particle()

            guessed_theta = self.transform_helper.angle_normalize( guessed_theta )
            
            print(f"guessed x: {guessed_x}     y: {guessed_y}     theta: {guessed_theta}")
            
            # Get position tuple and orientation quaternion tuple from guessed x,y,theta
            guessed_pos = (guessed_x, guessed_y, 0.0)
            guessed_rot = quaternion_from_euler(0.0, 0.0, guessed_theta)

            # Create pose from position & orientation tuples
            self.robot_pose = self.transform_helper.convert_translation_rotation_to_pose( guessed_pos, guessed_rot )

            self.transform_helper.fix_map_to_odom_transform(self.robot_pose, self.odom_pose)


    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample in helper_functions.py.
        """
        print(self.particles.W)
        resampled_indices = draw_integer_sample( self.particles.W, self.n_particles)

        print(resampled_indices)
        print(len(resampled_indices))

        self.particles.X     = np.array([ self.particles.X[i] for i in resampled_indices])
        self.particles.Y     = np.array([ self.particles.Y[i] for i in resampled_indices])
        self.particles.Theta = np.array([ self.particles.Theta[i] for i in resampled_indices])
        self.particles.W     = np.ones(self.n_particles)
        self.particles.normalize()


    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        print("updating initial pose")
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)


    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud by sampling from a normal distribution.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used """  
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)

        # Unpack initial x,y,theta
        [initial_x, initial_y, initial_theta] = xy_theta
        print(f"Initial x: {initial_x}. Initial y: {initial_y}. Initial theta: {initial_theta}")

        self.particles.initialize_from_hypothesis(initial_x, initial_y, initial_theta,
                                                  self.init_x_dev, self.init_y_dev, self.init_theta_dev)


    def publish_particles(self, timestamp):
        particles_conv = self.particles.as_poses()
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