import roslibpy


client = roslibpy.Ros(host='localhost', port=8080)


def on_ready():
    print(client.is_connected)


def sub_joy(msg):
    print(msg)


if __name__ == '__main__':
    # client.run()
    # print(client.is_connected)
    client.on_ready(on_ready)

    listener = roslibpy.Topic(client, '/joy', 'sensor_msgs/Joy')
    listener.subscribe(sub_joy)

    client.run_forever()
    client.terminate()
