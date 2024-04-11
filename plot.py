#!/usr/bin/env python
import rosbag
import matplotlib.pyplot as plt
from tqdm import tqdm

pos = []
vel = []

# Open the bag file
with rosbag.Bag('2024-04-11-09-26-15.bag') as bag:
    # Iterate over messages
    total_messages = bag.get_message_count()
    progress_bar = tqdm(total=total_messages, desc="Processing messages", unit="message")
    for topic, msg, t in bag.read_messages(topics=['/vel', '/pos']):
        # Check the topic and process the message accordingly
        if topic == '/vel':
            # Process velocity messages
            vel.append(float(msg.data))
        elif topic == '/pos':
            # Process position messages
            pos.append(float(msg.data))
        progress_bar.update(1)
    progress_bar.close()
    if (len(pos) == len(vel)):
        plt.plot(pos, vel, linestyle='-', color='red')
        # Adding labels and title
        plt.xlabel('Posición')
        plt.ylabel('Velocidad')
        plt.title('Diagrama de fase')

        # Displaying the plot
        plt.grid(True)
        plt.show()
    else:
        print("not equal length")
        print(len(pos))
        print(len(vel))