"""
Tracks the current state of the elevator and associated cost function
"""

import math
from collections import defaultdict

import numpy as np

### Cost Functions ###

# Cost function weightins in declaration order
_COST_WEIGHTS = [0.4, 0.5, 0.1]
assert(sum(_COST_WEIGHTS) == 1)

def out_time_cost(t):
    """
    Cost function term for time spent waiting outside car
    """
    return t ** 2

def in_time_cost(t):
    """
    Cost function term for time spent waiting inside car
    """
    return t ** 0.5

def pass_count_cost(c):
    """
    Cost function term for number times car passed passenger without stopping
    """
    return c ** 3

def overall_cost(passengers):
    total_cost = 0
    for p in passengers:
        total_cost += _COST_WEIGHTS[0] * out_time_cost(p.out_time) + \
            _COST_WEIGHTS[1] * in_time_cost(p.in_time) + \
            _COST_WEIGHTS[2] * pass_count_cost(p.pass_count)
    return total_cost


### Elevator Environment ###
class Passenger:
    """
    out_time: Number of iterations spent outside elevator (waiting for car)
    in_time: Number of iterations spent inside elevator (waiting in car)
    pass_count: Number of times passed by a car
    destination: Destination floor
    in_car: Boolean representing whether or not they are in a car
    """
    def __init__(self, dest):
        self.out_time = 0
        self.in_time = 0
        self.pass_count = 0
        self.destination = dest
        self.in_car = False

class Floor:
    """
    number: Which floor number, starting at ground floor 0
    up_button: State of up button (0 unpressed, 1 pressed)
    down_button: State of down button (0 unpressed, 1 pressed)

    lambda_func: Time variant lambda function
    """
    def __init__(self, floor_num, num_floors, lambda_func):
        self.number = floor_num
        self.num_floors = num_floors
        self.up_button = 0
        self.down_button = 0

        #Passenger arrival parameters
        self.max_flow_rate = max_flow_rate
        self.peak_time = peak_time
        self.lambda_func = lambda_func

        #Mapped by destination floor to passenger list
        self.passengers = []

    def run_iteration(self, cur_time, cars):
        """
        Adds passengers to the floor
        """
        passenger_count = np.random.poisson(self.lambda_func(cur_time))
        new_passengers = []
        valid_floors = list(range(self.num_floors))
        valid_floors.pop(self.number)

        for i in range(passenger_count):
            new_pass = Passenger(np.random.choice(valid_floors))
            new_passengers.append(new_pass)
        self.add_passengers(self, new_passengers)

        cur_floor_cars = [ car for car in cars if car.cur_floor == self.number ]
        if not cur_floor_cars:
            return

        up_pass = [ psgr for psgr in self.passengers if psgr.destination > self.number ]
        down_pass = [ psgr for psgr in self.passengers if psgr.destination < self.number ]
        
        #Up pass will prefer cars heading up but will also take no heading
        #Down pass prefers cars heading down but also takes no heading
        up_cars = [ car for car in cur_floor_cars if car.heading == 0 ]
        down_cars = [ car for car in cur_floor_cars if car.heading == 1 ]

        #TODO: BOARD PASSENGERS

    def add_passengers(self, new_passengers):
        """
        Add passengers who have arrived at a floor and are waiting
        """
        if type(new_passengers) == list:
            self.passengers.extend(new_passengers)
        elif type(new_passengers) == dict or type(new_passengers) == defaultdict:
            for dest, pass_list in new_passengers:
                self.passengers.extend(pass_list)
        else:
            raise ValueError("new_passengers must be one of type list or dict")

    def update_passengers(self):
        """
        Updates passengers on this floor
        """

    def reset(self):
        self.up_button = 0
        self.down_button = 0
        self.passengers = defaultdict(list)

class Car:
    """
    number: Car number, used as identifier
    cur_floor: Which floor the car is currently on
    buttons_pushed: Vector of binary states of buttons for each floor inside car
    """
    def __init__(self, car_num, num_floors, default_floor=0):
        self.number = car_num
        self.default_floor = default_floor
        self.cur_floor = default_floor
        self.num_floors = num_floors
        self.buttons_pushed = [0 for i in range(num_floors)]

        #0 up, 1 down
        self.heading = 0

        #Used for calculating cost
        self.passengers = []

        #Only know one passenger per call
        #If multiple people board on the same floor going to the same destination
        #car can only know for sure that one person boarded
        self.known_passengers = []

    def action(self, act_num):
        # 0: Move up
        # 1: Move down
        # 2: Stay (up)
        # 3: Stay (down)
        if act_num == 0 and self.cur_floor < (self.num_floors - 1):
            self.cur_floor += 1
            self.heading = 1
        elif act_num == 1 and self.cur_floor > 0:
            self.cur_floor -= 1
            self/heading = 2
        elif act_num == 2:
            self.heading = 0
        elif act_num == 3:
            self.heading = 1
        return self.cur_floor

    def board_passenger(self, passenger):
        self.passengers.append(passenger)
        if self.buttons_pushed(passenger.destination) == 0:

            #Add passenger to set of known only if they push a new button
            self.known_passengers.append(passenger)

    def reset(self):
        self.cur_floor = self.default_floor
        self.buttons_pushed = [0 for i in range(self.num_floors)]
        self.passengers = []
        self.known_passengers = []

class Elevator:
    """
    floors: List of floors in elevator system
    cars: List of cars in elevator system
    """
    def __init__(self, num_floors, num_cars, lambda_funcs):
        assert(num_floors == len(lambda_funcs))

        self.num_floors = num_floors
        self.num_cars = num_cars
        self.floors = [ Floor(i, self.num_floors, lambda_funcs[i]) for i in range(num_floors) ]
        self.cars = [ Car(i, num_floors) for i in range(num_cars) ]

        self.nfeats = num_cars * 2 + 2

    def run_iteration(self, cur_time, actions):
        """
        Runs single state iteration, modifying state of floors
        """
        for car, act in zip(self.cars, actions):
            car.action(act)
        for floor in self.floors:
            floor.run_iteration(cur_time, self.cars)

    def reset(self):
        for floor in self.floors:
            floor.reset()
        for car in self.cars:
            car.reset()

    def state(self):
        """
        Returns n x m feature matrix where n is the number of floors and
        m is the number of features
        m = (2 + 3) for each car + 2 for outside buttons
        """

        #Outside button feature vectors
        floor_button_up = [ f.up_button for f in self.floors ]
        floor_button_down = [ f.down_button for f in self.floors ]
        feature_vecs = [floor_button_up, floor_button_down]
        for car in self.cars:

            #Feature vectors per car
            loc_vec = [ 0 if i != car.cur_floor else 1
                    for i in range(self.num_floors) ]
            feature_vecs.append(loc_vec)
            feature_vecs.append(car.buttons_pushed)

            #Heading vectors per car
            heading_vecs = [ np.zeros(self.num_floors) if i == car.heading \
                             else np.ones(self.num_floors) for i in range(2) ]
            feature_vecs.extend(heading_vecs)

        #Features in cols
        return np.array(feature_vecs).T

    def total_cost(self):
        cost = 0
        for car in self.cars:
            cost += overall_cost(car.passengers)
        return cost

    def known_cost(self):
        cost = 0
        for car in self.cars:
            cost += overall_cost(car.known_passengers)
        return cost

def gen_lambda_func(peak_time, peak_duration, max_lambda, base_prop):
    """
    peak_time: Peak discrete period number
    peak_duration: How long on either side peak lasts
    max_lambda: Maximum flow rate
    base_prop: Base proportion of max lambda
    """
    #Peaks around peak_time for an hour on either side then tapers to base
    sin_func = lambda x : math.sin(x - (math.pi / 2)) + 1
    peak_start = peak_time - peak_duration
    peak_end = peak_time + peak_duration

    #This is essentially one period of a sin function
    def lambda_func(cur_time):
        if abs(cur_time - peak_time) > peak_duration:
            return base_prop * max_lambda
        elif cur_time < peak_time:
            time_from_start = cur_time - peak_start
            period_prop = time_from_start / peak_duration
            sin_val = sin_func(period_prop * math.pi)
            return max_lambda * ((sin_val / 2 * (1 - base_prop)) + base_prop)
        else:
            time_from_end = peak_end - cur_time
            period_prop = 1 - time_from_end / peak_duration
            sin_val = sin_func(math.pi + period_prop * math.pi)
            return max_lambda * ((sin_val / 2 * (1 - base_prop)) + base_prop)

    return lambda_func

