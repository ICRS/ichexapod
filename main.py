"""

The layout of the robot's legs is assumed to be as follows:

    [0]     [1]
    [2]  C  [3]
    [4]     [5]
"""
import collections
import datetime
import math
import enum
import time

import numpy as np
from numpy import linalg

# TODO: This should be renamed to something else.
import hello

def artificial_sleep(begin_time, amount):
    now = time.time()
    if now - begin_time >= amount:
        return
    sleep(amount - (now - begin_time))


def rotate_motors(angles):
    pass


class Cycle(enum.IntEnum):
    FORWARD = 0
    INVERT = 1

    def flip(self):
        if self == FORWARD:
            return self.INVERT
        else:
            return self.FORWARD

class RobotState(enum.IntEnum):
    STOP = 0
    FORWARD = 1


DELTA_TIME = 0.2  # in seconds in long trajectories

# These are in mm
DELTA_MOVEMENT = 10  # chunks of 1cm every command
LIFT_HEIGHT = 50  # Lift legs up that distance
BUBBLE_RADIUS = 75
JOYSTICK_THRESHOLD = 10  # What units is this?

def cap_magnitude(direction, magnitude):
    if linalg.norm(direction) < magnitude:
        return direction

    return direction / linalg.norm(direction) * magnitude


def read_vector_from_controller():
    # TODO: Interface with code that actually read commands from the XBox
    #       controller
    pass


class Solver(object):

    def __init__(self, usb):
        self.primary = [np.zeros(3) for _ in range(6)]
        self.support = [np.zeros(3) for _ in range(6)]
        self.cycle = Cycle.FORWARD
        self._usb = usb

    def _move(self, primary, support):

        if self.cycle == Cycle.FORWARD:
            first = zip(self.primary, primary)
            second = zip(self.support, support)
        else:
            first = zip(self.support, support)
            second = zip(self.primary, primary)

        intents = [
                first[0],
                second[0],
                second[1],
                first[1],
                first[2],
                second[2]
        ]
        angles = [0] * 6

        for i, intent in enumerate(intents):
            ((sx, sy, sz), (tx, ty, tz)) = intent
            assert isinstance(sx, float)
            assert isinstance(sy, float)
            assert isinstance(sz, float)
            assert isinstance(tx, float)
            assert isinstance(ty, float)
            assert isinstance(tz, float)
            angles[i] = hello.call_motors(i, sx, sy, sz, tx, ty, tz)

        # This is UART, hence async. This won't block.
        i = 0
        for tuple in angles:
            for angle in tuple:
                self._usb.setTarget(i, angle)  # TODO(fyq14): [angle] to quadmicroseconds.
                i += 1

        # Update at the end
        for i, p in enumerate(primary):
            if p is not None:
                self.primary[i] = p

        for i, p in enumerate(support):
            if p is not None:
                self.support[i] = p

        self.primary = list(primary)
        self.support = list(support)

    def cycle(self, stop_at_center):

        # 1. Lift the first 3 legs
        self._move(
                primary=[
                    positions[0] + np.array([0, 0, LIFT_HEIGHT]),
                    positions[1] + np.array([0, 0, LIFT_HEIGHT]),
                    positions[2] + np.array([0, 0, LIFT_HEIGHT])
                ],
                support=self.support,
        )

        difference_ratio = math.sqrt(
                (BUBBLE_RADIUS ** 2) - (LIFT_HEIGHT ** 2)) / BUBBLE_RADIUS


        begin_time = time.time()

        # 2. forward stride
        while True:
            if any(linalg.norm(p, 2) >= BUBBLE_RADIUS - 5.0 for p in self.positions):
                break

            delta_front = difference_ratio * DELTA_MOVEMENT
            delta_back = DELTA_MOVEMENT

            if stop_at_center:
                forward_target = np.zero([0, 0, LIFT_HEIGHT])
                backward_target = np.zero([0, 0, 0])
            else:
                forward_target = np.zero([0, difference_ratio * BUBBLE_RADIUS, LIFT_HEIGHT])
                backward_target = np.zero([0, -BUBBLE_RADIUS, 0])

            # HACK(fyq14): This assumes all thre have exactly the same
            #              position in their respective coordinates.
            delta_front = cap_magnitude(
                    forward_target - self.primary[0], magnitude=delta_front
            )
            delta_back = cap_magnitude(
                    backward_target - self.support[0], magnitude=delta_back
            )

            self._move(
                    primary=[v + delta_front for v in self.primary],
                    support=[v + delta_back for v in self.support]
            )

            artificial_sleep(begin_time=begin_time, amount=DELTA_TIME)
            begin_time = time.time()

        # 3. Put the legs down
        self._move(
                primary=[v - np.array([0, 0, LIFT_HEIGHT]) for v in self.primary],
                support=self.support
        )
        sleep(0.1)
        self.cycle = self.cycle.flip()

def main():
    solver = Solver()
    current_state = RobotState.STOP

    while True:
        # Use netcat or something to read for commands
        joystick_vector = read_vector_from_controller()

        if linalg.norm(joystick_vector) < JOYSTICK_THRESHOLD:
            if current_state == RobotState.STOP:
                pass
            elif current_state == RobotState.FORWARD:
                solver.cycle(stop_at_center=True)
                current_state = RobotState.STOP
        else:
            solver.cycle(stop_at_center=False)

if __name__ == "__main__":
    main()
