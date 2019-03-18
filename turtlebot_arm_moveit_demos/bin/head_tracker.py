#!/usr/bin/env python

import rospy, sys, tf
from sensor_msgs.msg import RegionOfInterest, CameraInfo
import moveit_commander
from math import *
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation
from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy


GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

GRIPPER_FRAME = 'gripper_link'
GRIPPER_JOINT_NAMES = ['gripper_joint']
GRIPPER_EFFORT = [1.0]
GRIPPER_PARAM = '/gripper_controller'

REFERENCE_FRAME = '/base_link'
ARM_BASE_FRAME = '/arm_base_link'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('moveit_demo')
	rate = rospy.get_param("~rate", 20)
        r = rospy.Rate(rate)
        tick = 1.0 / rate
               
        self.gripper_opened = [rospy.get_param(GRIPPER_PARAM + "/max_opening") ]
        self.gripper_closed = [rospy.get_param(GRIPPER_PARAM + "/min_opening") ]
        self.gripper_neutral = [rospy.get_param(GRIPPER_PARAM + "/neutral") ]
        
        self.gripper_tighten = rospy.get_param(GRIPPER_PARAM + "/tighten") 

        # We need a tf listener to convert poses into arm reference base
        self.tf_listener = tf.TransformListener()
        
        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('target_pose', PoseStamped, queue_size=10)

        # Create a dictionary to hold object colors
        self.colors = dict()

	# Set a flag to indicate when the target has been lost
        self.target_visible = False

	# Set a timer to determine how long a target is no longer visible
        target_lost_timer = 0.0

	 # Get a lock for updating the self.move_cmd values
        #self.lock = thread.allocate_lock()

        # Initialize the move group for the right arm
        arm = MoveGroupCommander(GROUP_NAME_ARM)

        # Initialize the move group for the right gripper
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()

        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.04)
        arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)

        # Set the right arm reference frame
        arm.set_pose_reference_frame(REFERENCE_FRAME)

        # Allow 5 seconds per planning attempt
        arm.set_planning_time(5)

        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 3

        # Set a limit on the number of place attempts
        max_place_attempts = 3
        rospy.loginfo("Scaling for MoveIt timeout=" + str(rospy.get_param('/move_group/trajectory_execution/allowed_execution_duration_scaling')))

        # Give the scene a chance to catch up
        rospy.sleep(2)

        # Give the scene a chance to catch up
        rospy.sleep(1)

        # Start the arm in the "arm_up" pose stored in the SRDF file
        rospy.loginfo("Set Arm: right_up")
        arm.set_named_target('right_up')
        if arm.go() != True:
            rospy.logwarn("  Go failed")
        rospy.sleep(2)

        # Move the gripper to the closed position
        rospy.loginfo("Set Gripper: Close " + str(self.gripper_closed ) )
        gripper.set_joint_value_target(self.gripper_closed)   
        if gripper.go() != True:
            rospy.logwarn("  Go failed")
        rospy.sleep(2)
        
        # Move the gripper to the neutral position
        rospy.loginfo("Set Gripper: Neutral " + str(self.gripper_neutral) )
        gripper.set_joint_value_target(self.gripper_neutral)
        if gripper.go() != True:
            rospy.logwarn("  Go failed")
        rospy.sleep(2)

        # Move the gripper to the open position
        rospy.loginfo("Set Gripper: Open " +  str(self.gripper_opened))
        gripper.set_joint_value_target(self.gripper_opened)
        if gripper.go() != True:
            rospy.logwarn("  Go failed")
        rospy.sleep(2)

	# Move the arm to the test position
        rospy.loginfo("Set Arm: test")
        arm.set_named_target('test')
        if arm.go() != True:
            rospy.logwarn("  Go failed")
        rospy.sleep(2)


	#
	#	For head tracker
	#

	 # Subscribe to camera_info topics and set the callback
        self.image_width = self.image_height = 0
        rospy.Subscriber('camera_info', CameraInfo, self.get_camera_info, queue_size=1)

	
	# Subscribe to roi topics and set the callback
        self.roi_subscriber = rospy.Subscriber('roi', RegionOfInterest, self.set_joint_cmd, queue_size=1)
        
        rospy.loginfo("Ready to track target.")

	while not rospy.is_shutdown():
            # Acquire the lock
            #self.lock.acquire()
            #This thing will execute when there is no target
            try:
                # If we have lost the target, stop the servos
                if not self.target_visible:
		    rospy.loginfo("Set Arm Resting While Waiting For The Target.")
                    arm.set_named_target('resting')
        	    arm.go()
		    rospy.sleep(2)	

                    
                    # Keep track of how long the target is lost
                    target_lost_timer += tick
                else:
		    arm.set_named_target('forward')
		    arm.go()
		    rospy.sleep(2)
		
		    rospy.loginfo("Waving To The Left.")
		    arm.set_named_target('forward_left')
        	    arm.go()
		    rospy.sleep(2)

		    rospy.loginfo("Waving To The Right.")
		    arm.set_named_target('forward_right')
        	    arm.go()
		    rospy.sleep(2)
	
                    self.target_visible = False
                    target_lost_timer = 0.0
                             
            
            finally:
                # Release the lock
                #self.lock.release()
		'''if self.target_visible == False:
		        rospy.loginfo("Waving To The Left.")
			arm.set_named_target('forward_left')
			arm.go()

			rospy.loginfo("Waving To The Right.")
			arm.set_named_target('forward_right')
			arm.go()'''

            r.sleep()
		



	#
	#	end of head tracker
	#

        # Return the arm to the "resting" pose stored in the SRDF file (passing through right_up)
        arm.set_named_target('right_up')
        arm.go()
        
        arm.set_named_target('resting')
        arm.go()

        # Open the gripper to the neutral position
        gripper.set_joint_value_target(self.gripper_neutral)
        gripper.go()

        rospy.sleep(1)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()

        # Exit the script
        moveit_commander.os._exit(0)



   #for the roi
    def set_joint_cmd(self, msg):
        
        try:
            # If we receive an ROI messages with 0 width or height, the target is not visible
            if msg.width == 0 or msg.height == 0:
                self.target_visible = False
                return
            
            # If the ROI stops updating this next statement will not happen
	    else: 		
		self.target_visible = True
		rospy.loginfo("Target Found, Start to Bow Down.")
		

	 
	finally:
            # Release the lock
            #self.lock.release()
		#r.sleep()
		rospy.loginfo("Hello.")
		rospy.sleep(1)
   

    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()

        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES

        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()

        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions

        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT

        tp.time_from_start = rospy.Duration(1.0)

        # Append the goal point to the trajectory points
        t.points.append(tp)

        # Return the joint trajectory
        return t

   
			



    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()

        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        # The vector is relative to the gripper frame
        g.direction.header.frame_id = GRIPPER_FRAME

        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired

        return g

    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects, grasp_opening=[0]):
        # Initialize the grasp object
        g = Grasp()

        # Set the pre-grasp and grasp postures appropriately;
        # grasp_opening should be a bit smaller than target width
        g.pre_grasp_posture = self.make_gripper_posture(self.gripper_opened)
        g.grasp_posture = self.make_gripper_posture(grasp_opening)

        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.01, 0.1, [1.0, 0.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0.0, -1.0, 1.0])

        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped

        # Pitch angles to try
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.4, -0.4]

        # Yaw angles to try; given the limited dofs of turtlebot_arm, we must calculate the heading
        # from arm base to the object to pick (first we must transform its pose to arm base frame)
        target_pose_arm_ref = self.tf_listener.transformPose(ARM_BASE_FRAME, initial_pose_stamped)
        x = target_pose_arm_ref.pose.position.x
        y = target_pose_arm_ref.pose.position.y

        self.pick_yaw = atan2(y, x)   # check in make_places method why we store the calculated yaw
        yaw_vals = [self.pick_yaw]

        # A list to hold the grasps
        grasps = []

        # Generate a grasp for each pitch and yaw angle
        for yaw in yaw_vals:
            for pitch in pitch_vals:
                # Create a quaternion from the Euler angles
                q = quaternion_from_euler(0, pitch, yaw)

                # Set the grasp pose orientation accordingly
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]

                # Set and id for this grasp (simply needs to be unique)
                g.id = str(len(grasps))

                # Set the allowed touch objects to the input list
                g.allowed_touch_objects = allowed_touch_objects

                # Don't restrict contact force
                g.max_contact_force = 0

                # Degrade grasp quality for increasing pitch angles
                g.grasp_quality = 1.0 - abs(pitch)

                # Append the grasp to the list
                grasps.append(deepcopy(g))

        # Return the list
        return grasps

    # Generate a list of possible place poses
    def make_places(self, init_pose):
        # Initialize the place location as a PoseStamped message
        place = PoseStamped()

        # Start with the input place pose
        place = init_pose

        # A list of x shifts (meters) to try
        x_vals = [0, 0.005, -0.005] #, 0.01, -0.01, 0.015, -0.015]

        # A list of y shifts (meters) to try
        y_vals = [0, 0.005, -0.005, 0.01, -0.01] #, 0.015, -0.015]

        # A list of pitch angles to try
        pitch_vals = [0] #, 0.005, -0.005, 0.01, -0.01, 0.02, -0.02]

        # A list to hold the places
        places = []

        # Generate a place pose for each angle and translation
        for pitch in pitch_vals:
            for dy in y_vals:
                for dx in x_vals:
                    place.pose.position.x = init_pose.pose.position.x + dx
                    place.pose.position.y = init_pose.pose.position.y + dy
    
                    # Yaw angle: given the limited dofs of turtlebot_arm, we must calculate the heading from
                    # arm base to the place location (first we must transform its pose to arm base frame)
                    target_pose_arm_ref = self.tf_listener.transformPose(ARM_BASE_FRAME, place)
                    x = target_pose_arm_ref.pose.position.x
                    y = target_pose_arm_ref.pose.position.y
                    yaw = atan2(y, x) - self.pick_yaw;
                    # Note that we subtract the yaw we calculated for pick, as the picked object "carries"
                    # with him the orientation of the arm at pickup time. More details in this moveit-users
                    # group thread:  https://groups.google.com/forum/#!topic/moveit-users/-Eie-wLDbu0 
    
                    # Create a quaternion from the Euler angles
                    q = quaternion_from_euler(0, pitch, yaw)
    
                    # Set the place pose orientation accordingly
                    place.pose.orientation.x = q[0]
                    place.pose.orientation.y = q[1]
                    place.pose.orientation.z = q[2]
                    place.pose.orientation.w = q[3]
    
                    # Append this place pose to the list
                    places.append(deepcopy(place))

        # Return the list
        return places

    # Set the color of an object
    def setColor(self, name, r, g, b, a=0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()

        # Set the id to the name given as an argument
        color.id = name

        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a

        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff
        p.is_diff = True

        # Append the colors from the global color dictionary
        for color in self.colors.values():
            p.object_colors.append(color)

        # Publish the scene diff
        self.scene_pub.publish(p)


    #To get the face target by calculating the width and the height
    def get_camera_info(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height

if __name__ == "__main__":
	try:
  		MoveItDemo()
	except KeyboardInterrupt:
		print "Shutting down Arm Movement and Face Tracking."

		# Shut down MoveIt cleanly
		moveit_commander.roscpp_shutdown()

		# Exit the script
		moveit_commander.os._exit(0)

		
