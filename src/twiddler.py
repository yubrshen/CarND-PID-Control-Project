import random
import numpy as np
import matplotlib.pyplot as plt

# ------------------------------------------------
#
# this is the Robot class
#

class Robot(object):
    def __init__(self, length=20.0):
        """
        Creates robot and initializes location/orientation to 0, 0, 0.
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, x, y, orientation):
        """
        Sets a robot coordinate.
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise):
        """
        Sets the noise parameters.
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def move(self, steering, distance = 1, tolerance=0.001, max_steering_angle=np.pi / 4.0):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift

        # Execute motion
        turn = np.tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)


############## ADD / MODIFY CODE BELOW ####################
# ------------------------------------------------------------------------
#
# run - does a single control run

from copy import deepcopy

class Twiddler(object):
    def __init__(self, p, d, i):
        """
        Create twiddler for event-driven twiddle process.
        """
        self.counter_event = 0
        self.error = 0
        self.best_error = -1
        self.params = [p, d, i]
        self.best_params = deepcopy(self.params)
        self.delta_params = [0.1, 0.1, 0.1]
        self.adjustment_allowance = 0.002
        self.next_param = -1
        self.previous_adjustment = 1
        self.settled = False
        self.STABILIZATION = 50
        self.COLLECTION = 50

    def reset(self):
        self.counter_event = -1  # so that the next meaningful count starts with 0
        self.error = 0

    def process(self, cte):

        if self.counter_event < self.STABILIZATION:  # just keep running
            pass
        else:
            self.error += cte * cte      # start to collect the error
        # end of if self.counter_event < self.STABILIZATION
        adjustment_needed = (self.counter_event == self.COLLECTION + self.STABILIZATION)
        if adjustment_needed:
            self.settled = (sum(self.delta_params) < self.adjustment_allowance)
            if not self.settled:
                self.evaluate_and_adjust()
                self.reset()
            else:
                self.params = deepcopy(self.best_params)
            # end of if not self.settled
        # end of if counter_event == COLLECTION + STABILIZATION
        self.counter_event += 1
        return adjustment_needed

    def evaluate_and_adjust(self):
        #from IPython.core.debugger import Tracer; Tracer()()
        if (self.best_error == -1) or (self.error < self.best_error):  # improved
            if (self.best_error == -1):
                self.best_error = self.error # this staement cannot be factored out,
                # as best_error cannot be changed before test against its current value
                self.start_adjust_next_parameter(-1)
            else: # (error < best_error)
                self.best_error = self.error
                self.best_params = deepcopy(self.params)                # commit the exploration
                self.delta_params[self.next_param] *= 1.1
                self.start_adjust_next_parameter(self.next_param)
            # end of if (best_error == -1) or (best_error < error):  # improved
        else:                       # worsened
            self.params = deepcopy(self.best_params)  # abandon the previous experiment
            if (self.previous_adjustment == 1):  # the previous adjustment is increase, then there is decrease to try
                self.params[self.next_param] += -self.delta_params[self.next_param]
                self.previous_adjustment = -1
            else:                   # previous_adjustment == -1, already exhausted the adjustment for the parameter
                self.delta_params[self.next_param] *= 0.9
                self.start_adjust_next_parameter(self.next_param)
            # end of if (self.previous_adjustment == 1)
        # end of (best_error == -1)...

    def start_adjust_next_parameter(self, current):
        self.next_param = (current +1) % len(self.params)
        self.params[self.next_param] += self.delta_params[self.next_param]
        self.previous_adjustment = 1

def make_robot():
    """
    Resets the robot back to the initial position and drift.
    You'll want to call this after you call `run`.
    """
    robot = Robot()
    robot.set(0, 1, 0)
    robot.set_steering_drift(10 / 180 * np.pi)
    return robot

def compute_feedback(params, err, prev_err, int_err):
    diff_err = err - prev_err
    int_err += err
    prev_err = err
    steer = -params[0] * err - params[1] * diff_err - params[2] * int_err
    return prev_err, int_err, steer

large_num = 100000              # the number of overall event process transactions

prev_cte = 0
int_cte = 0

#twiddler = Twiddler(0.2, 3.0, 0.004)
p = random.randint(0, 100)/100
d = random.randint(0, 100)/10
i = random.randint(0, 100)/1000
twiddler = Twiddler(p, d, i)
robot = make_robot()

for i in range(large_num):
    cte = robot.y
    prev_cte, int_cte, steer =  compute_feedback(twiddler.params, cte, prev_cte, int_cte)  # compute new steer
    speed = 1 # may be used to adjust the speed, the distance in unit time interval
    robot.move(steer, speed)
    cte = robot.y
    adjustment_needed = twiddler.process(cte)
    if twiddler.settled:
        break
    # end of if settled
    if adjustment_needed:
        robot = make_robot()
    # end of if settled
# end of for in range(large_num)

x_trajectory = []
y_trajectory = []
robot = make_robot()
prev_cte = 0
int_cte = 0
print("delat_params: ", twiddler.delta_params)
print("params: ", twiddler.params)
print("best_params", twiddler.best_params)
print("best_error: ", twiddler.best_error)
print("breaking i: ", i)
for i in range(1000):
    cte = robot.y
    prev_cte, int_cte, steer =  compute_feedback(
        twiddler.params, cte, prev_cte, int_cte)  # compute new steer
    robot.move(steer)
    x_trajectory.append(robot.x)
    y_trajectory.append(robot.y)
n = len(x_trajectory)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 8))
ax1.plot(x_trajectory, y_trajectory, 'g', label='Twiddle PID controller')
ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')

print("THE END")
