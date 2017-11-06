import datetime
import json
import numpy as np

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
        self.x = 0.0
        self.y = 0.0
        self.heading_rad = 0.0
        self.hitch_rad = 0.0
        self.v = 0.0
        self.steering_rad = 0.0
        self.sample_time = 0.0
        self.sample_number = 0

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
        self.x = 0.0
        self.y = 0.0
        self.heading_rad = 0.0
        self.hitch_rad = 0.0
        self.v = 1.0
        self.steering_rad = 10 * (np.pi / 180)
        self.sample_time = 0.1
        self.sample_number = 50

        self.log('[Control]\t v: ', self.v, ', steering: ', self.steering_rad * (180 / np.pi))

        self.log('[INITIAL]\t x: ', self.x, ', y: ', self.y,
                 ', heading: ', self.heading_rad * (180/np.pi),
                 ', hitch: ', self.hitch_rad * (180/np.pi))

        # Perform loop for given period of time
        for i in range(0, self.sample_number):
            self.loop_state_space()
            self.log('[Step', i, ']\t x: ', self.x, ', y: ', self.y,
                     ', heading: ', self.heading_rad * (180 / np.pi),
                     ', hitch: ', self.hitch_rad * (180 / np.pi))
        self.log('Completed...')

    """
    Loop part of state space model
    """
    def loop_state_space(self):
        x_dot = self.v * np.cos(self.heading_rad)
        y_dot = self.v * np.sin(self.heading_rad)
        heading_dot = self.v * self.L * np.tan(self.steering_rad)
        hitch_p1 = (self.L2 + self.L1 * np.cos(self.hitch_rad)) / self.L2
        hitch_p2 = (np.sin(self.hitch_rad) / self.L2) * self.v
        hitch_dot = hitch_p1 * heading_dot - hitch_p2

        self.x += x_dot * self.sample_time
        self.y += y_dot * self.sample_time
        self.heading_rad += heading_dot * self.sample_time
        self.hitch_rad += hitch_dot * self.sample_time

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
