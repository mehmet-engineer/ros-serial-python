#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse

class ForceSensorHandler:
    def __init__(self):
        rospy.init_node('serial_force_publisher')

        self.publish_topic = '/force_data'
        self.publisher = rospy.Publisher(self.publish_topic, Float32, queue_size=10)

        self.port = '/dev/ttyUSB0'
        self.baud = 9600
        
        try: 
            self.serial_port = serial.Serial(self.port, self.baud)
            rospy.loginfo("Serial port opened. Port: " + self.port + ", Baud: " + str(self.baud))
        except serial.SerialException as e:
            rospy.logerr("Failed to open serial port: " + str(e))

        self.publish_hz = 100
        self.rate = rospy.Rate(self.publish_hz)

        calibration_topic = "/calibrate_force_sensor"
        self.service_server = rospy.Service(calibration_topic, Empty, self.calibrate_sensor)
        #self.service_client = rospy.ServiceProxy(calibration_topic, Empty)


    def calibrate_sensor(self, req):
        rospy.loginfo("Calibrating force sensor...")
        
        try:
            calibrate_data = "1"
            self.serial_port.write(calibrate_data.encode())

        except serial.SerialException as e:
            rospy.logerr("Failed to send calibration command: " + str(e))
        
        return EmptyResponse()

    def run(self):
        rospy.loginfo("Force sensor publisher started.")
        
        while not rospy.is_shutdown():
            
            try:
                if self.serial_port.in_waiting > 0:
                
                    try:
                        line = self.serial_port.readline().decode('utf-8').strip()
                        force_value = float(line)

                        rospy.loginfo("Force: " + str(force_value) + " N")
                        self.publisher.publish(force_value)
                    
                    except ValueError:
                        rospy.logwarn("Received invalid data, skipping this value.")
            
            except:
                rospy.logwarn("Failed to communicate from serial port.")
                    
            self.rate.sleep()

        self.serial_port.close()
        rospy.loginfo("Serial port closed.")


if __name__ == '__main__':
    
    try:
        sensor = ForceSensorHandler()
        sensor.run()
        
    except rospy.ROSInterruptException:
        pass
