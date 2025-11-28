import roslibpy
import numpy as np
from db_helper import insert_lidar_data

# ROS 연결
client = roslibpy.Ros(host='localhost', port=9090)
client.run()
print('Connected to ROS!')

cmd_vel_pub = roslibpy.Topic(
    client,
    '/turtle1/cmd_vel',
    'geometry_msgs/msg/Twist'
)

listener = roslibpy.Topic(
    client,
    '/lidar_data_mock',
    'sensor_msgs/msg/LaserScan'
)

def callback(message):
    ranges = np.array(message["ranges"])

    front = np.r_[ranges[350:360], ranges[0:10]]
    left  = ranges[80:100]
    right = ranges[260:280]

    front_dist = np.mean(front)
    left_dist  = np.mean(left)
    right_dist = np.mean(right)

    safe_dist = 0.5

    if front_dist < safe_dist:
        if left_dist > right_dist:
            action = "turn_left"
            cmd_vel_pub.publish({
                'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': 1.0}
            })
        else:
            action = "turn_right"
            cmd_vel_pub.publish({
                'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': -1.0}
            })
    else:
        action = "go_forward"
        cmd_vel_pub.publish({
            'linear': {'x': 0.5, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        })

    print("front:", round(front_dist, 2))
    print("left :", round(left_dist, 2))
    print("right:", round(right_dist, 2))
    print("action:", action)

    # MySQL에 저장
    insert_lidar_data(ranges, action)

try:
    listener.subscribe(callback)
    while client.is_connected:
        pass
except KeyboardInterrupt:
    pass
finally:
    listener.unsubscribe()
    client.terminate()
    print('Disconnected.')