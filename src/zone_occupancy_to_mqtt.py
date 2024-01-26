#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from gtec_msgs.msg import ZoneOccupancy
import paho.mqtt.client as mqtt


class MQTTPublisher:
    def __init__(self, broker_address, topic, username, password):
        self.broker_address = broker_address
        self.topic = topic
        self.username = username
        self.password = password
        self.client = mqtt.Client()

    def connect(self):
        self.client.username_pw_set(self.username, self.password)
        self.client.connect(self.broker_address, 1883, 60)

    def publish(self, zone_id, count, target_ids):
        zone_id_no_spaces = zone_id.replace(" ", "_")
        message = '{{"count": {}, "target_ids": {}}}'.format(count, list(target_ids))
        self.client.publish(self.topic+zone_id_no_spaces, message)


class ZoneOccupancySubscriber:
    def __init__(self, mqtt_publisher):
        self.mqtt_publisher = mqtt_publisher
        self.inside = 0

    def callback(self, data):
        zone_id = data.zoneId
        count = data.count
        target_ids = data.targetIds

        if count != 0:
            self.mqtt_publisher.publish(zone_id, count, target_ids)


def main():
    rospy.init_node('zone_occupancy_to_mqtt')

    broker_address = rospy.get_param('~broker_address')
    mqtt_topic = rospy.get_param('~mqtt_topic')
    zone_occupancy_topic = rospy.get_param('~publish_zone_ocuppancy_topic')
    username = rospy.get_param('~mqtt_username')
    password = rospy.get_param('~mqtt_password')

    mqtt_publisher = MQTTPublisher(broker_address, mqtt_topic, username, password)
    mqtt_publisher.connect()

    zone_occupancy_subscriber = ZoneOccupancySubscriber(mqtt_publisher)

    rospy.Subscriber(zone_occupancy_topic, ZoneOccupancy, zone_occupancy_subscriber.callback)

    rospy.spin()


if __name__ == '__main__':
    main()
