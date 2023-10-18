import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use('fivethirtyeight')

x_vals = []
y_vals = []

index = count()

def animate(i):
    data = pd.read_csv('/home/drcl/Desktop/mjbot_2023/src/mjbot_vision/vision/log.csv')
    x = data['runtime']
    width = data['w']
    height = data['h']
    width_diff = data['w_diff']
    height_diff = data['h_diff']
    fall = data['fall']

    plt.cla()

    plt.ylim([-1000, 1000])
    plt.xlabel('time (sec)')
    plt.ylabel('size')
    
    # plt.plot(x, width, label='width')
    plt.plot(x, height, label='height')
    # plt.plot(x, width_diff, label='width_diff')
    plt.plot(x, height_diff, label='height_diff')
    plt.plot(x, fall, label='fall')

    plt.legend(loc='upper left')
    plt.tight_layout()

    print('animate func called')

ani = FuncAnimation(plt.gcf(), animate, interval=10)

plt.tight_layout()
plt.show()
