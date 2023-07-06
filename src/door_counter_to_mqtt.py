#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from gtec_msgs.msg import DoorCounterEvent
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

    def publish(self, a_type, a_value):
        message = '{{"type": "{}", "value": {}}}'.format(a_type, a_value)
        self.client.publish(self.topic, message)


class DoorCounterSubscriber:
    def __init__(self, mqtt_publisher):
        self.mqtt_publisher = mqtt_publisher
        self.inside = 0

    def callback(self, data):
        lth = data.lth
        htl = data.htl
        diff = lth - htl

        if diff != 0:
            a_type = "in" if diff > 0 else "out"
            a_value = abs(diff)
            self.mqtt_publisher.publish(a_type, a_value)


def main():
    rospy.init_node('door_counter_subscriber')

    broker_address = rospy.get_param('~broker_address')
    mqtt_topic = rospy.get_param('~mqtt_topic')
    door_counter_topic = rospy.get_param('~publish_door_counter_topic')
    username = rospy.get_param('~mqtt_username')
    password = rospy.get_param('~mqtt_password')

    mqtt_publisher = MQTTPublisher(broker_address, mqtt_topic, username, password)
    mqtt_publisher.connect()

    door_counter_subscriber = DoorCounterSubscriber(mqtt_publisher)

    rospy.Subscriber(door_counter_topic, DoorCounterEvent, door_counter_subscriber.callback)

    rospy.spin()


if __name__ == '__main__':
    main()
