#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Header
from gtec_msgs.msg import ZoneOccupancy
import signal
import time

# Define a flag to indicate if Ctrl+C has been pressed
ctrl_c_pressed = False
start_time = None

def signal_handler(sig, frame):
    global ctrl_c_pressed
    rospy.loginfo("Ctrl+C detected. Cleaning up...")
    ctrl_c_pressed = True

def get_seconds():
    global start_time

    if start_time is None:
        # If start_time is not set, this is the first call
        start_time = time.time()
        return 0  # or any other initial value you prefer
    else:
        # Calculate the elapsed time in seconds since the first call
        return time.time() - start_time
        
        
def seconds_to_mmss(the_seconds):
    minutes = int(the_seconds // 60.0)
    seconds = int(the_seconds % 60.0)
    return f"{minutes:02d}:{seconds:02d}"
    
            
def message_callback(data, file):
    count = data.count

        
    if (int(count)>0):
        message_time = get_seconds()
        zone_id = data.zoneId
        zone_id_no_spaces = zone_id.replace(" ", "_")
        file.write(f"{message_time},{zone_id_no_spaces},{count}\n") 
    
def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('zone_occupancy_to_file')
    rate = rospy.Rate(10)

    
    file_name = rospy.get_param('~zone_occupancy_file_name')
    zone_occupancy_topic = rospy.get_param('~publish_zone_ocuppancy_topic')


    file = open(file_name, 'w')
    file.write("seconds,area,count\n")
    
    rospy.Subscriber(zone_occupancy_topic, ZoneOccupancy, message_callback, callback_args=(file))

    while not ctrl_c_pressed and not rospy.is_shutdown():
        rate.sleep()
        
        
    file.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass