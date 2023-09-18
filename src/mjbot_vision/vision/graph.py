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
    data = pd.read_csv('log.csv')
    x = data['run_time']
    dx = data['width']
    dy = data['height']
    width_diff = data['width_diff']
    height_diff = data['height_diff']
    fall = data['fall_detection']

    plt.cla()

    plt.ylim([-7000, 7000])
    plt.xlabel('time (sec)')
    plt.ylabel('size')
    
    # plt.plot(x, dx, label='width')
    # plt.plot(x, dy, label='height')
    plt.plot(x, width_diff, label='width_diff')
    plt.plot(x, height_diff, label='height_diff')
    plt.plot(x, fall, label='fall')

    plt.legend(loc='upper left')
    plt.tight_layout()

    print('animate func called')

ani = FuncAnimation(plt.gcf(), animate, interval=10)

plt.tight_layout()
plt.show()
