#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf
import numpy
from std_msgs.msg import String
from gazebo_msgs.msg import ContactsState
import math

class MarkerBasics(object):
    def __init__(self, topic_id):
        marker_topic = '/marker_basic_'+topic_id
        self.marker_objectlisher = rospy.Publisher(marker_topic, Marker, queue_size=1)
        self.rate = rospy.Rate(25)
        self.init_marker(index=0)

    def init_marker(self, index=0):
        self.marker_object = Marker()
        self.change_frame(frame="/world", ns="dogbot", index=0)
        self.marker_object.type = Marker.ARROW
        self.marker_object.action = Marker.ADD

        self.change_position(x=0.0, y=0.0, z=0.0)
        self.change_orientation(pitch=0.0, yaw=0.0)
        self.change_scale()
        self.change_colour(R=1.0, G=0.0, B=0.0)

        # If we want it for ever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = rospy.Duration(0)

    def change_orientation(self, pitch, yaw):
        """
        Roll doesnt make any sense in an arrow
        :param pitch: Up Down. We clip it to values [-1.5708,1.5708]
        :param yaw: Left Right , No clamp
        :return:
        """
        pitch = numpy.clip(pitch, -1.5708,1.5708)

        q = tf.transformations.quaternion_from_euler(0.0, pitch, yaw)

        self.marker_object.pose.orientation.x = q[0]
        self.marker_object.pose.orientation.y = q[1]
        self.marker_object.pose.orientation.z = q[2]
        self.marker_object.pose.orientation.w = q[3]

    def change_position(self, x, y, z):
        """
        Position of the starting end of the arrow
        :param x:
        :param y:
        :param z:
        :return:
        """

        my_point = Point()
        my_point.x = x
        my_point.y = y
        my_point.z = z
        self.marker_object.pose.position = my_point
        #rospy.loginfo("PositionMarker-X="+str(self.marker_object.pose.position.x))

    def change_colour(self, R, G, B):
        """
        All colours go from [0.0,1.0].
        :param R:
        :param G:
        :param B:
        :return:
        """

        self.marker_object.color.r = R
        self.marker_object.color.g = G
        self.marker_object.color.b = B
        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0

    def change_scale(self, s_x=1.0, s_y=0.1, s_z=0.1):
        """

        :param s_x:
        :param s_y:
        :param s_z:
        :return:
        """

        self.marker_object.scale.x = s_x
        self.marker_object.scale.y = s_y
        self.marker_object.scale.z = s_z

    def start(self):
        pitch = -0.7
        yaw = 0.0
        s_x = 1.0

        while not rospy.is_shutdown():
            #self.change_orientation(pitch=pitch,yaw=yaw)
            self.change_scale(s_x=s_x)
            self.marker_objectlisher.publish(self.marker_object)
            self.rate.sleep()
            s_x -= 0.01

    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)

    def pressure_to_wavelength_to_rgb(self, pressure, min_pressure=-50.0, max_pressure=50.0, gamma=0.8):

        '''This converts a given wavelength of light to an
        approximate RGB color value. The wavelength must be given
        in nanometers in the range from 380 nm through 750 nm
        (789 THz through 400 THz).

        Based on code by Dan Bruton
        http://www.physics.sfasu.edu/astro/color/spectra.html
        '''

        wavelength = self.translate(value=pressure,
                                   leftMin=min_pressure, leftMax=max_pressure,
                                   rightMin=380, rightMax=750)

        wavelength = float(wavelength)

        rospy.logdebug("pressure=" + str(pressure))
        rospy.logdebug("wavelength="+str(wavelength))

        if wavelength >= 380 and wavelength <= 440:
            attenuation = 0.3 + 0.7 * (wavelength - 380) / (440 - 380)
            R = ((-(wavelength - 440) / (440 - 380)) * attenuation) ** gamma
            G = 0.0
            B = (1.0 * attenuation) ** gamma
        elif wavelength >= 440 and wavelength <= 490:
            R = 0.0
            G = ((wavelength - 440) / (490 - 440)) ** gamma
            B = 1.0
        elif wavelength >= 490 and wavelength <= 510:
            R = 0.0
            G = 1.0
            B = (-(wavelength - 510) / (510 - 490)) ** gamma
        elif wavelength >= 510 and wavelength <= 580:
            R = ((wavelength - 510) / (580 - 510)) ** gamma
            G = 1.0
            B = 0.0
        elif wavelength >= 580 and wavelength <= 645:
            R = 1.0
            G = (-(wavelength - 645) / (645 - 580)) ** gamma
            B = 0.0
        elif wavelength >= 645 and wavelength <= 750:
            attenuation = 0.3 + 0.7 * (750 - wavelength) / (750 - 645)
            R = (1.0 * attenuation) ** gamma
            G = 0.0
            B = 0.0
        else:
            R = 0.0
            G = 0.0
            B = 0.0

        return R, G, B

    def change_frame(self, frame="/world", ns="dogbot", index=0):
        """

        :param frame:
        :return:
        """

        self.marker_object.header.frame_id = frame
        self.marker_object.header.stamp = rospy.get_rostime()
        self.marker_object.ns = ns
        self.marker_object.id = index


    def update_marker(self, frame, ns, index, position, orientation, pressure, min_pressure=0.0, max_pressure=10.0):
        """

        :param position: [X,Y,Z] in the world frame
        :param pressure: Magnitude
        :param orientation: [Pitch,Yaw]
        :return:
        """
        #self.change_frame(frame=frame, ns=ns, index=index)
        self.change_position(x=position[0], y=position[1], z=position[2])
        self.change_orientation(pitch=orientation[0], yaw=orientation[1])
        self.change_scale(s_x = pressure)

        R,G,B = self.pressure_to_wavelength_to_rgb(pressure=pressure,
                                                   min_pressure=min_pressure,
                                                   max_pressure=max_pressure,
                                                   gamma=0.8)

        rospy.logdebug("R,G,B=["+str(R)+", "+str(G)+", "+str(B)+"]")

        self.change_colour(R=R, G=G, B=B)

        self.marker_objectlisher.publish(self.marker_object)



class FootPressureInfo(object):

    def __init__(self):

        self.min_pressure = 0.0
        self.max_pressure = 2.0
        self.markerbasics_object_front_left_foot = MarkerBasics(topic_id="front_left_foot")
        self.markerbasics_object_front_right_foot = MarkerBasics(topic_id="front_right_foot")
        self.markerbasics_object_back_left_foot = MarkerBasics(topic_id="back_left_foot")
        self.markerbasics_object_back_right_foot = MarkerBasics(topic_id="back_right_foot")

        rospy.Subscriber("/dogbot/back_left_contactsensor_state", ContactsState, self.contact_callback_back_left_foot)
        rospy.Subscriber("/dogbot/back_right_contactsensor_state", ContactsState, self.contact_callback_back_right_foot)
        rospy.Subscriber("/dogbot/front_left_contactsensor_state", ContactsState, self.contact_callback_front_left_foot)
        rospy.Subscriber("/dogbot/front_right_contactsensor_state", ContactsState, self.contact_callback_front_right_foot)


    def contact_callback_front_right_foot(self, data):
        """

        :param data:
        :return:
        """
        foot_name = data.header.frame_id

        if len(data.states) >= 1:
            Fx = data.states[0].total_wrench.force.x
            Fy = data.states[0].total_wrench.force.y
            Fz = data.states[0].total_wrench.force.z
            pressure = math.sqrt(pow(Fx,2)+pow(Fy,2)+pow(Fz,2))

            Px = data.states[0].contact_positions[0].x
            Py = data.states[0].contact_positions[0].y
            Pz = data.states[0].contact_positions[0].z

            pressure = pressure / 100.0
            # rospy.loginfo(str(foot_name) + "--->pressure =" + str(pressure))
            # rospy.loginfo(str(foot_name) + "Point =[" + str(pressure))

            index = 1

            self.markerbasics_object_front_right_foot.update_marker(frame=foot_name,
                                                   ns="dogbot",
                                                   index=index,
                                                   position=[Px, Py, Pz],
                                                   orientation=[-1.57, 0.0],
                                                   pressure=pressure,
                                                   min_pressure=self.min_pressure,
                                                   max_pressure=self.max_pressure)


        else:
            # No Contact
            pass


    def contact_callback_front_left_foot(self, data):
        """

        :param data:
        :return:
        """
        foot_name = data.header.frame_id

        if len(data.states) >= 1:
            Fx = data.states[0].total_wrench.force.x
            Fy = data.states[0].total_wrench.force.y
            Fz = data.states[0].total_wrench.force.z
            pressure = math.sqrt(pow(Fx,2)+pow(Fy,2)+pow(Fz,2))

            Px = data.states[0].contact_positions[0].x
            Py = data.states[0].contact_positions[0].y
            Pz = data.states[0].contact_positions[0].z

            pressure = pressure / 100.0


            index = 0

            self.markerbasics_object_front_left_foot.update_marker(frame=foot_name,
                                                   ns="dogbot",
                                                   index=index,
                                                   position=[Px, Py, Pz],
                                                   orientation=[-1.57, 0.0],
                                                   pressure=pressure,
                                                   min_pressure=self.min_pressure,
                                                   max_pressure=self.max_pressure)
        else:
            # No Contact
            pass

    def contact_callback_back_right_foot(self, data):
        """

        :param data:
        :return:
        """
        foot_name = data.header.frame_id

        if len(data.states) >= 1:
            Fx = data.states[0].total_wrench.force.x
            Fy = data.states[0].total_wrench.force.y
            Fz = data.states[0].total_wrench.force.z
            pressure = math.sqrt(pow(Fx,2)+pow(Fy,2)+pow(Fz,2))

            Px = data.states[0].contact_positions[0].x
            Py = data.states[0].contact_positions[0].y
            Pz = data.states[0].contact_positions[0].z

            pressure = pressure / 100.0
            #rospy.loginfo(str(foot_name) + "--->pressure =" + str(pressure))
            # rospy.loginfo(str(foot_name) + "Point =[" + str(pressure))

            index = 2

            self.markerbasics_object_back_right_foot.update_marker(frame=foot_name,
                                                   ns="dogbot",
                                                   index=index,
                                                   position=[Px, Py, Pz],
                                                   orientation=[-1.57, 0.0],
                                                   pressure=pressure,
                                                   min_pressure=self.min_pressure,
                                                   max_pressure=self.max_pressure)


        else:
            # No Contact
            pass


    def contact_callback_back_left_foot(self, data):
        """

        :param data:
        :return:
        """
        foot_name = data.header.frame_id

        if len(data.states) >= 1:
            Fx = data.states[0].total_wrench.force.x
            Fy = data.states[0].total_wrench.force.y
            Fz = data.states[0].total_wrench.force.z
            pressure = math.sqrt(pow(Fx,2)+pow(Fy,2)+pow(Fz,2))

            Px = data.states[0].contact_positions[0].x
            Py = data.states[0].contact_positions[0].y
            Pz = data.states[0].contact_positions[0].z

            pressure = pressure / 100.0

            index = 3

            self.markerbasics_object_back_left_foot.update_marker(frame=foot_name,
                                                   ns="dogbot",
                                                   index=index,
                                                   position=[Px, Py, Pz],
                                                   orientation=[-1.57, 0.0],
                                                   pressure=pressure,
                                                   min_pressure=self.min_pressure,
                                                   max_pressure=self.max_pressure)
        else:
            # No Contact
            pass




if __name__ == '__main__':
    rospy.init_node('footpressure_marker_node', anonymous=True)
    footpressure_object = FootPressureInfo()
    rospy.spin()

