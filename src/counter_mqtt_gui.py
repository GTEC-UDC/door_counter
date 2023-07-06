#!/usr/bin/env python3

import sys
import configparser
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout
from PyQt5.QtCore import Qt
import paho.mqtt.client as mqtt

class MQTTSubscriber:
    def __init__(self, broker, topic, update_label, username, password):
        self.broker = broker
        self.topic = topic
        self.update_label = update_label
        self.username = username
        self.password = password
        self.client = mqtt.Client()
        self.client.username_pw_set(username, password)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def connect(self):
        self.client.connect(self.broker, 1883, 60)
        self.client.loop_start()

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker")
        self.client.subscribe(self.topic)

    def on_message(self, client, userdata, msg):
        message = msg.payload.decode()
        print("Received message:", message)
        try:
            data = eval(message)
            a_type = data['type']
            a_value = int(data['value'])
            self.update_label(a_type, a_value)
        except (SyntaxError, KeyError, ValueError) as e:
            print("Error parsing message:", str(e))


class GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MQTT Subscriber")
        self.setGeometry(100, 100, 300, 200)

        self.label = QLabel("0", self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 48px;")

        self.caption_label = QLabel("People Inside", self)
        self.caption_label.setAlignment(Qt.AlignCenter)

        self.broker_edit = QLineEdit(self)
        self.broker_edit.setPlaceholderText("Broker Address")

        self.topic_edit = QLineEdit(self)
        self.topic_edit.setPlaceholderText("Topic")

        self.username_edit = QLineEdit(self)
        self.username_edit.setPlaceholderText("Username")

        self.password_edit = QLineEdit(self)
        self.password_edit.setPlaceholderText("Password")
        self.password_edit.setEchoMode(QLineEdit.Password)

        self.start_button = QPushButton("Start", self)
        self.start_button.clicked.connect(self.toggle_subscription)

        layout = QVBoxLayout(self)
        layout.addWidget(self.label)
        layout.addWidget(self.caption_label)
        layout.addWidget(self.broker_edit)
        layout.addWidget(self.topic_edit)
        layout.addWidget(self.username_edit)
        layout.addWidget(self.password_edit)
        layout.addWidget(self.start_button)

        self.mqtt_subscriber = None
        self.subscribed = False

        self.load_initial_values()

    def load_initial_values(self):
        config = configparser.ConfigParser()
        if config.read("init.cfg"):
            if "Settings" in config:
                settings = config["Settings"]
                if "broker_address" in settings:
                    self.broker_edit.setText(settings["broker_address"])
                if "mqtt_topic" in settings:
                    self.topic_edit.setText(settings["mqtt_topic"])
                if "mqtt_username" in settings:
                    self.username_edit.setText(settings["mqtt_username"])

    def toggle_subscription(self):
        if not self.subscribed:
            broker = self.broker_edit.text()
            topic = self.topic_edit.text()
            username = self.username_edit.text()
            password = self.password_edit.text()

            self.mqtt_subscriber = MQTTSubscriber(broker, topic, self.update_label, username, password)
            self.mqtt_subscriber.connect()

            self.save_initial_values()
            self.start_button.setText("Stop")
            self.subscribed = True
        else:
            self.mqtt_subscriber.disconnect()
            self.mqtt_subscriber = None

            self.start_button.setText("Start")
            self.subscribed = False

    def save_initial_values(self):
        config = configparser.ConfigParser()
        config["Settings"] = {
            "broker_address": self.broker_edit.text(),
            "mqtt_topic": self.topic_edit.text(),
            "mqtt_username": self.username_edit.text()
        }
        with open("init.cfg", "w") as config_file:
            config.write(config_file)

    def update_label(self, a_type, a_value):
        current_value = int(self.label.text())
        if a_type == "set":
            self.label.setText(str(a_value))
        elif a_type == "in":
            self.label.setText(str(current_value + a_value))
        elif a_type == "out":
            new_value = current_value - a_value
            self.label.setText(str(new_value if new_value >= 0 else 0))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = GUI()
    gui.show()
    sys.exit(app.exec())
