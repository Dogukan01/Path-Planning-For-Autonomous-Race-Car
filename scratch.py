import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

fig, ax = plt.subplots()
x = np.linspace(0, 10, 1000)
y = np.sin(x)
line, = ax.plot(x, y)

def update(frame):
    ax.set_xlim(frame/10, frame/10 + 2)
    return line,

ani = animation.FuncAnimation(fig, update, frames=100, interval=20, blit=False)
plt.show(block=False)
plt.pause(2)
