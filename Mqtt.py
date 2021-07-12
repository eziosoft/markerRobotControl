import paho.mqtt.client as mqtt
import time

mqtt_client = mqtt.Client()
mqtt_server_address = "192.168.0.19"  # my MQTT server
ch1 = 100
ch2 = 100
ch3 = 100
ch4 = 100


# Connect to MQTT server
def on_connect(client, userdata, flags, rc):
    client.subscribe("tank/out")
    print("Connected with result code " + str(rc))


def on_message(client, userdata, msg):
    # print(str(msg.payload[1]) + " " + str(msg.payload[2]))
    # print(msg.payload)
    print()


def connectMQTT():
    # mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    try:
        print("connecting to mqtt server " + str(mqtt_server_address))
        mqtt_client.connect(mqtt_server_address, 1883, 60)
        # mqtt_client.loop_forever(timeout=1.0, max_packets=1, retry_first_connection=False)
        mqtt_client.loop_start()

    except Exception as e:
        print("Unable to connect to MQTT server" + str(mqtt_server_address))

def disconnectMqtt():
    mqtt_client.loop_stop()
    mqtt_client.disconnect()

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def mqtt_control_robot(ch1, ch2, ch3, ch4):
    ch1 = clamp(ch1, 0, 200)
    ch2 = clamp(ch2, 0, 200)
    ch3 = clamp(ch3, 0, 200)
    ch4 = clamp(ch4, 0, 200)
    control_frame = [0x24, 4, int(ch1), int(ch2), int(ch3), int(ch4)]
    mqtt_client.publish('tank/in', bytearray(control_frame))


# connectMQTT()
# while True:
#     mqtt_control_robot(100, 100, 100, 100)
#     time.sleep(100)
