import elevator

### CONSTANTS FOR SETTING UP ELEVATOR ###

# Four actions, move up, move down, stay (head up), stay (head down)
N_ACTIONS = elevator.N_ACTIONS
NUM_FLOORS = 10
NUM_CARS = 1
MAX_LAMBDA = 5
FIRST_FLOOR_FUNC = elevator.gen_lambda_func(10, 5, MAX_LAMBDA, 0.7)
LAMBDA_FUNCS = [ lambda x : MAX_LAMBDA * 0.4 \
                for i in range(NUM_FLOORS - 1)]
LAMBDA_FUNCS.insert(0, FIRST_FLOOR_FUNC)

NUM_EPOCHS = 10
EPOCH_DURATION = 100
