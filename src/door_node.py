#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import ColorRGBA, Float32
from jsk_rviz_plugins.msg import OverlayText

from grid_occupancy import DoorGridHandler, DoorGridSize, DoorHandlerOptions, TargetPoint, DoorGrid, GridSize
from gtec_msgs.msg import RadarFusedPointStamped, DoorCounterEvent


if __name__ == "__main__":

    rospy.init_node('DoorNode', anonymous=True)
    rate = rospy.Rate(5)  # hz
    door_id = rospy.get_param('~door_id')
    target_positions_topic = rospy.get_param('~target_positions_topic')
    publish_door_counter_topic = rospy.get_param('~publish_door_counter_topic')
    cell_size = float(rospy.get_param('~cell_size'))
    grid_width = float(rospy.get_param('~grid_width'))
    grid_height = float(rospy.get_param('~grid_height'))
    grid_low_zone_height = float(rospy.get_param('~grid_low_zone_height'))
    grid_high_zone_height = float(rospy.get_param('~grid_high_zone_height'))

    grid_size = GridSize(width_meters=grid_width, height_meters=grid_height, cell_size=cell_size)
    door_grid_size = DoorGridSize(grid_size=grid_size, low_zone_height=grid_low_zone_height, high_zone_height=grid_high_zone_height)
    reduce_fun = lambda time_elapsed: time_elapsed * 10
    expansion_fun = lambda num_cells: 50/(num_cells +1)
    options = DoorHandlerOptions(reduce_fun=reduce_fun, expansion_fun=expansion_fun, complete_threshold=40, min_expansion_threshold=20, nearby_threshold=100, nearby_distance=0.3, delete_threshold=10, max_cost=150)
    
    door_handler = DoorGridHandler(options=options, grid_size=door_grid_size)
    


    #Reference grid to plot in rviz
    ref_grid = DoorGrid(door_grid_size=door_grid_size)
    #tp_points = ref_grid.getGridPositions()
    high_point = ref_grid.getGridPositionsHighzone()
    low_points = ref_grid.getGridPositionsLowzone()
    cloud_points = []
    for tp in high_point:
        cloud_points.append([tp.x, tp.y, 0, 100]) #Cost is in z value

    for tp in low_points:
        cloud_points.append([tp.x, tp.y, 0, 50]) #Cost is in z value
    
    
    header_point_cloud = Header()
    header_point_cloud.frame_id = "grid"
    fields_point_cloud =  [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('cost', 12, PointField.FLOAT32, 1 ),]

    pub_ref_grid = rospy.Publisher('/gtec/door/ref_grid', PointCloud2, queue_size=100)
    pub_grid_0 = rospy.Publisher('/gtec/door/'+ str(door_id), PointCloud2, queue_size=100)

    pub_overlay_text = rospy.Publisher('/gtec/door/text', OverlayText, queue_size=100)

    pub_door_counter = rospy.Publisher(publish_door_counter_topic, DoorCounterEvent, queue_size=100)


    tf_buffer = tf2_ros.Buffer(rospy.Duration(3.0)) #tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform("grid",
                                   "map", #source frame
                                   rospy.Time(0), #get the tf at first available time
                                   rospy.Duration(1.0))


    # def getTransformedPoint(point_msg, id):
    #     tf_pos = tf2_geometry_msgs.do_transform_point(point_msg, transform)
    #     return TargetPoint(tf_pos.point.x, tf_pos.point.y, tf_pos.point.z, 0, id)


    # for n in range(8):
    #     pos_handler = lambda pos: door_handler.newDetection(getTransformedPoint(pos,n))
    #     rospy.Subscriber(str(target_positions_topic)+'/'+str(n), PointStamped, pos_handler)  

    def getTransformedPoint(point_msg, id):
        tf_pos = tf2_geometry_msgs.do_transform_point(point_msg, transform)
        return TargetPoint(tf_pos.point.x, tf_pos.point.y, tf_pos.point.z, 0, id)


    pos_handler = lambda pos: door_handler.newDetection(getTransformedPoint(pos,pos.targetId))
    rospy.Subscriber(str(target_positions_topic), RadarFusedPointStamped, pos_handler) 


    total_low_to_high = 0
    total_high_to_low = 0

    print("=========== GTEC Door Node ============")

    while not rospy.is_shutdown():
        header_point_cloud.stamp = rospy.Time.now()
        point_cloud_grid = pc2.create_cloud(header_point_cloud, fields_point_cloud, cloud_points)
        pub_ref_grid.publish(point_cloud_grid)

        if (len(door_handler.door_grids)>0):
            door_grid_0 = door_handler.door_grids[0]
            door_0_points = door_grid_0.occupancy_grid.getGridPositions()
            cloud_0_points = []
            for tp in door_0_points:
                cloud_0_points.append([tp.x, tp.y, 0, tp.z]) #Cost is in z value
            header_point_cloud.stamp = rospy.Time.now()
            point_cloud_grid_0 = pc2.create_cloud(header_point_cloud, fields_point_cloud, cloud_0_points)
            pub_grid_0.publish(point_cloud_grid_0)
            
        low_to_high, high_to_low = door_handler.loop()
        total_low_to_high = total_low_to_high + low_to_high
        total_high_to_low = total_high_to_low + high_to_low

        if (low_to_high != 0 or high_to_low !=0):
            print(f'Low_to_high: {low_to_high} High_to_low: {high_to_low}')
            door_counter_event = DoorCounterEvent()
            door_counter_event.header = header_point_cloud
            door_counter_event.lth = low_to_high
            door_counter_event.htl = high_to_low
            pub_door_counter.publish(door_counter_event)


        
        text_msg = OverlayText()
        text_msg.text = f'Low_to_high: {total_low_to_high} \n\nHigh_to_low: {total_high_to_low}'
        text_msg.width = 400
        text_msg.height = 600
        #text.height = 600
        text_msg.left = 10
        text_msg.top = 10
        text_msg.text_size = 25
        text_msg.line_width = 2
        text_msg.font = "DejaVu Sans Mono"
        text_msg.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
        text_msg.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
        pub_overlay_text.publish(text_msg)

        

        rate.sleep()