import rosbag
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from scipy import optimize
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from itertools import cycle


# bag = rosbag.Bag('2022-07-24-20-59-24.bag')
bag = rosbag.Bag('2022-07-25-11-20-00.bag')

topics = bag.get_type_and_topic_info()[1].keys()
print(topics)


t_s = [msg.pose.position.x for (topic, msg, t) in bag.read_messages(topics=['/tf/world/cf1'])]

print(t_s)