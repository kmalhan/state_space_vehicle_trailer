import datetime
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation

"""
Class implementation of state space model for vehicle and trailer
"""


class StateSpaceModel:

    def __init__(self):
        self.log('Start State Space Modeling...')
        # Class variables
        self.L = 0.0
        self.L1 = 0.0
        self.L2 = 0.0
        self.sample_number = 50
        self.x = np.zeros(self.sample_number + 1)
        self.y = np.zeros(self.sample_number + 1)
        self.heading_rad = np.zeros(self.sample_number + 1)
        self.hitch_rad = np.zeros(self.sample_number + 1)
        self.v = 0.0
        self.steering_rad = 0.0
        self.sample_time = 0.0

    """
    Load vehicle configuration setting
    """
    def load_vehicle_config(self):
        with open('vehicle_config.json') as config_file:
            config = json.load(config_file)

        self.L = config["vehicle_wheelbase_m"]
        self.L1 = config["hitch_length_m"]
        self.L2 = config["trailer_length_m"]

    """
    Initial setup of state space model
    """
    def setup_state_space(self):
        # Setup Initial Stage
        self.x[0] = 0.0
        self.y[0] = 0.0
        self.heading_rad[0] = 0.0
        self.hitch_rad[0] = 0.0
        self.v = 1.0
        self.steering_rad = 10 * (np.pi / 180)
        self.sample_time = 0.1
        self.sample_number = 50
        # TODO: Update array size based on this new sample size

        self.log('[Control]\t v: ', self.v, ', steering: ', self.steering_rad * (180 / np.pi))

        self.log('[INITIAL]\t x: ', self.x[0], ', y: ', self.y[0],
                 ', heading: ', self.heading_rad[0] * (180/np.pi),
                 ', hitch: ', self.hitch_rad[0] * (180/np.pi))

        # Perform loop for given period of time
        for i in range(0, self.sample_number):
            self.loop_state_space(i)
            self.log('[Step', i+1, ']\t x: ', self.x[i+1], ', y: ', self.y[i+1],
                     ', heading: ', self.heading_rad[i+1] * (180 / np.pi),
                     ', hitch: ', self.hitch_rad[i+1] * (180 / np.pi))
        self.log('Completed...')

    """
    Loop part of state space model
    """
    def loop_state_space(self, it):
        x_dot = self.v * np.cos(self.heading_rad[it])
        y_dot = self.v * np.sin(self.heading_rad[it])
        heading_dot = self.v * self.L * np.tan(self.steering_rad)
        hitch_p1 = (self.L2 + self.L1 * np.cos(self.hitch_rad[it])) / self.L2
        hitch_p2 = (np.sin(self.hitch_rad[it]) / self.L2) * self.v
        hitch_dot = hitch_p1 * heading_dot - hitch_p2

        self.x[it+1] = self.x[it] + x_dot * self.sample_time
        self.y[it+1] = self.y[it] + y_dot * self.sample_time
        self.heading_rad[it+1] = self.heading_rad[it] + heading_dot * self.sample_time
        self.hitch_rad[it+1] = self.hitch_rad[it] + hitch_dot * self.sample_time

    """
    Animate animation sub-function
    """
    def animate_movement(self, i):
        self.vehicle.set_width(1.0)
        self.vehicle.set_height(1.0)
        self.vehicle.set_xy([self.x[i], self.y[i]])
        self.vehicle._angle = -np.rad2deg(self.heading_rad[i])
        return self.vehicle, self.trailer,

    """
    Animate init sub-function
    """
    def init_movement(self):
        self.ax.add_patch(self.vehicle)
        self.ax.add_patch(self.trailer)
        return self.vehicle, self.trailer,

    """
    Visualization
    """
    def visualization(self):
        fig = plt.figure()
        plt.axis('equal')
        plt.grid()
        self.ax = fig.add_subplot(111)
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
        self.vehicle = patches.Rectangle((0, 0), 0, 0, fc='y')
        self.trailer = patches.Rectangle((0, 0), 0, 0, fc='b')

        anim = animation.FuncAnimation(fig, self.animate_movement,
                                init_func=self.init_movement,
                                frames=self.sample_number+1,
                                interval=250,
                                blit=True)
        plt.show()

    """
    Print output with current time
    """
    def log(*args):
        msg = ' '.join(map(str, [datetime.datetime.now(), '>'] + list(args)))
        print(msg)
        with open('../log.txt', 'at') as fd:
            fd.write(msg + '\n')

if __name__ == "__main__":
    state_space_model = StateSpaceModel()
    state_space_model.load_vehicle_config()
    state_space_model.setup_state_space()
    state_space_model.visualization()
