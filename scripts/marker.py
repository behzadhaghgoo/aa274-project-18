#!/usr/bin/env python

'''
DESCRIPTION: Creates an array of markers that are published on the /markers topic
These markers can then be visualized in Rviz. 
Currently create a circular marker at the robot's position. Can be extended to add 
other circular markers of different colors and shapes for objects such as food, 
stop signs, puddles, etc.
'''

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Pose2D
from std_msgs.msg import Header, ColorRGBA
from gazebo_msgs.msg import ModelStates
import tf

#----Module level variables---
#Robot marker definitions
ROBOT_SIZE = (0.16, 0.16, 0.16)
ROBOT_COLOR = (1.0, 0.0, 0.0, 0.8)
ROBOT_FRAME = "base_footprint"
ROBOT_POSITION_IN_FRAME = (0.0, 0.0, 0.0)

#Food marker definitions example
FOOD_SIZE = (0.16, 0.16, 0.16)
FOOD_COLOR = (0.0, 1.0, 0.0, 0.8)
FOOD_FRAME = "map"
FOOD_POSITION_IN_FRAME = (1.0, 1.0, 0.0)

class MarkerPublisher:

	def __init__(self):
		rospy.init_node('RvizMarkers', anonymous=True)

		#Publisher to publish markers to Rviz
		self.Markerpub = rospy.Publisher('/markers', MarkerArray, queue_size=10) 
		self.NavPub = rospy.Publisher('/NavRequest', Pose2D, queue_size = 10) 

        rospy.Subscriber('/detector/object', DetectedObject, self.object_detected):
        rospy.Subscriber('/delivery_request', String, self.request_subscriber):

		# current state
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.z = 1

		# goal state
		self.x_g = 0.0
		self.y_g = 0.0
		self.theta_g = 0.0

		# marker array
		self.marker_array = MarkerArray()
		self.numObjects = 0
		self.markers = []

        # Transfrom from camera frame to real world
        self.trans_listener = tf.TransformListener()

		#Create Robot marker
		self.createMarker("turtlebot", ROBOT_FRAME, ROBOT_POSITION_IN_FRAME, ROBOT_COLOR, ROBOT_SIZE)

		#Create example food marker (Will add functionality for automatically adding
		# food markers later)
		# self.createMarker(FOOD_FRAME, FOOD_POSITION_IN_FRAME, FOOD_COLOR,FOOD_SIZE)

		#Initialize Navigation dictionary with home's location and confidence (x,y,theta,confidence)
		self.NavDict = { 'home' : (0, 0, 0, 1) }

	'''
	Function: 		object_detected
	Description: 	Callback for detector/object subscriber. '''
    def object_detected(self, message):
        if message.id > 43 and message.id < 64 and message.confidence > 0.90

			#Bounding box parameters (From camera message)
			theta_l = message.thetaleft
            theta_r = message.thetaright
            dist = message.distance

			#Find camera's position and rotation relative to the map frame
			origin_frame = "/camera"
            (translation, rotation) = self.trans_listener.lookupTransform(origin_frame, '/map', rospy.Time(0))
            x = translation[0]
            y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            theta = euler[2]

			#See if we already have an object with this name recorded. If not, add it to NavDict
			marker_name = message.name
			if marker_name in not self.NavDict:
				 #Add object to our NavDict
            	self.NavDict[message.name] = (x,y,theta,message.confidence)

				#Create a marker
				self.createMarker(marker_name, FOOD_FRAME, (x, y, 0), FOOD_COLOR, FOOD_SIZE)

			#Otherwise, see if we want to update the object's location (perhaps using the confidence levels)
			else:
				pass

	'''
	Function: 		request_subscriber
	Description: 	Receives food order requests, looks up the food item's locations, and publishes
					the location as a Pose2D message. '''
    def request_subscriber(self, message)
        try:
            (x, y, theta, conf) = NavDict[message.name]
            rqt = Pose2D()
            rqt.x = x
            rqt.y = y
            rqt.theta = theta
            self.NavPub(rqt)
        except:
            pass


	'''
	Function: 		createMarkers
	Description: 	Adds a new marker to self.markers. Right now just adds
					a cylinder with the given parameters.
					EXAMPLE PARAMETERS
					frame = "map"
					color = [r,g,b,a]
					pos_in_frame = [0,0,0]
					scale = [0.16,0.16,0.16]	'''
	def createMarker(self, name, frame, id, pos_in_frame, color,scale):
		#Create our new marker
		NewMarker = Marker(
					type=3, #Cylinder
					id = id
                    ns = name
					lifetime=rospy.Duration(1.5),
					pose=Pose(Point(*pos_in_frame), Quaternion(0, 0, 0, 1)),
					scale=Vector3(*scale),
					header=Header(frame_id=frame), 
					color=ColorRGBA(*color),
					)
		#Increase our marker object count
		self.numObjects += 1

		#Add marker to our list of active markers
		self.markers.append(NewMarker)

	#Not implemented yet. Deletes markers if desired
	def deleteMarker(self, marker_num):
		pass
			
	#Construct MarkerArray and publish
	def publish_markers(self):
		#TODO: Find out a way to append new markers directly to MarkerArray
		self.marker_array = MarkerArray(markers = self.markers)
		self.pub.publish(self.marker_array)

	#Run function
	def run(self):
		rospy.loginfo("Starting Rviz Marker Node")
		rate = rospy.Rate(5)
		while not rospy.is_shutdown():
			self.publish_markers()
			rate.sleep()

#In other nodes do:
# from markers import returnNavDict
# NavDict = returnNavDict
def returnNavDict():
    return NavDict


if __name__ == '__main__':
	try:
		MarkerPub = MarkerPublisher()
		MarkerPub.run()
        object_detected()
        request_subscriber()
	except rospy.ROSInterruptException:
		pass

