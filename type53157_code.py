import os
import subprocess
import numpy as np
import sqlite3
from time import sleep
import pandas as pd
from filelocker import FileLock
from filelock import FileLock as FL

thisModule = os.path.splitext(os.path.basename(__file__))[0]
pre = "_trnSYS"
# Initialization: function called at TRNSYS initialization
# ----------------------------------------------------------------------------------------------------------------------
act_path = "action.txt"
obs_path = "observation.txt"
params_path = 'others.txt'

lock_path = ".lock"
flag = 'EndOfSimFlag'
is_busy = 'busy'
# state = ['iteration_running', 'iteration_ready']
state = 'iteration_ready'
state = FileLock(state)
# lock = FileLock(lock_path)
act_lock = FL(act_path + lock_path)
obs_lock = FL(obs_path + lock_path)
is_busy = FL(is_busy + lock_path)
integral = 0
previous_error = 0


def Initialization(TRNData):
    # mode = TRNData[thisModule]["inputs"][4]
    # # This model has nothing to initialize
    # if mode == 1:  # RL
    if (os.path.isfile(is_busy.lock_file + lock_path)):
        os.remove(is_busy.lock_file + lock_path)
    if (os.path.isfile(flag)):
        os.remove(flag)
        pass

    return

# StartTime: function called at TRNSYS starting time (not an actual time step, initial values should be reported)
# ----------------------------------------------------------------------------------------------------------------------


def StartTime(TRNData):

    # Define local short names for convenience (this is optional)

    # fp = open(state[0], 'w')
    # fp.close()
    global integral, previous_error

    # unlock observation
    # lock.release()
    # observation = TRNData[thisModule]["inputs"]  # [0]
    mode = TRNData[thisModule]["inputs"][4]

    # Error = Set Point – Process Variable
    error = TRNData[thisModule]["inputs"][0] - \
        TRNData[thisModule]["inputs"][1]
    integral = 0  # integral + error
    derivative = error - previous_error
    previous_error = error
    ctrl = 0
    if mode == 0:  # Use PID

        [Kp, Ki, Kd] = [TRNData[thisModule]["inputs"][4], TRNData[thisModule]
                        ["inputs"][5], TRNData[thisModule]["inputs"][6]]
        ctrl = Kp*error + Ki*integral + Kd*derivative

    elif mode == 1:  # RL mode
        observation = [error, integral, derivative]
        with obs_lock:
            np.savetxt(obs_path, np.array(observation))
            # other parameters
            # energy_consumed# weight_ctrl# weight_energy
            params = [TRNData[thisModule]["inputs"][8], TRNData[thisModule]
                      ["inputs"][9], TRNData[thisModule]["inputs"][10]]

            np.savetxt(params_path, np.array(params))
        pass
        # stepNo = TRNData[thisModule]["current time step number"]
        state.acquire()
        # # Set outputs in TRNData
    TRNData[thisModule]["outputs"][0] = ctrl
    return


# Iteration: function called at each TRNSYS iteration within a time step
# ----------------------------------------------------------------------------------------------------------------------
def Iteration(TRNData):
    global integral, previous_error

    mode = TRNData[thisModule]["inputs"][4]

    # Error = Set Point – Process Variable
    error = TRNData[thisModule]["inputs"][0] - \
        TRNData[thisModule]["inputs"][1]
    integral = integral + error
    derivative = error - previous_error
    previous_error = error

    ctrl = 0
    if mode == 0:  # Use PID

        [Kp, Ki, Kd] = [TRNData[thisModule]["inputs"][5], TRNData[thisModule]
                        ["inputs"][6], TRNData[thisModule]["inputs"][7]]
        ctrl = Kp*error + Ki*integral + Kd*derivative
        # Calculate the outputs

        # stepNo = TRNData[thisModule]["current time step number"]

        # # Set outputs in TRNData
        # TRNData[thisModule]["outputs"][0] = ctrl
        # Calculate the outputs
    elif mode == 1:
        while state.is_lock():
            continue
        observation = [error, integral, derivative]
        max_ = TRNData[thisModule]["inputs"][2]
        min_ = TRNData[thisModule]["inputs"][3]
        with is_busy:
            with obs_lock:
                np.savetxt(obs_path, np.array(observation))
                # other parameters
                params = [
                    TRNData[thisModule]['inputs'][8],  # energy_consumed
                    TRNData[thisModule]['inputs'][9],  # weight_ctrl
                    TRNData[thisModule]['inputs'][10],  # weight_energy
                ]
                np.savetxt(params_path, np.array(params))

            # read action
            # sleep(0.1)
            with act_lock:
                action = np.loadtxt(act_path).tolist()

                ctrl = scaling(action, min_, max_)
        pass

    stepNo = TRNData[thisModule]["current time step number"]

    # Set outputs in TRNData
    TRNData[thisModule]["outputs"][0] = ctrl
    # Set outputs in TRNData
    # TRNData[thisModule]["outputs"][0] = y
    # stepNo = TRNData[thisModule]["current time step number"]

    return

# EndOfTimeStep: function called at the end of each time step, after iteration and before moving on to next time step
# ----------------------------------------------------------------------------------------------------------------------


def EndOfTimeStep(TRNData):

    # This model has nothing to do during the end-of-step call
    # os.rename(state[0], state[1])
    # lock.release()
    state.acquire()
    # sleep(0.1)
    return


# LastCallOfSimulation: function called at the end of the simulation (once) - outputs are meaningless at this call
# ----------------------------------------------------------------------------------------------------------------------
def LastCallOfSimulation(TRNData):

    # NOTE: TRNSYS performs this call AFTER the executable (the online plotter if there is one) is closed.
    # Python errors in this function will be difficult (or impossible) to diagnose as they will produce no message.
    # A recommended alternative for "end of simulation" actions it to implement them in the EndOfTimeStep() part,
    # within a condition that the last time step has been reached.
    #
    # Example (to be placed in EndOfTimeStep()):
    #
    # stepNo = TRNData[thisModule]["current time step number"]
    # nSteps = TRNData[thisModule]["total number of time steps"]
    # if stepNo == nSteps-1:     # Remember: TRNSYS steps go from 0 to (number of steps - 1)
    #     do stuff that needs to be done only at the end of simulation
    fp = open(flag, 'w')
    fp.close()
    return


def scaling(ctrl_sig, min_sig, max_sig):
    # action ==>
    # [-1 1] ==> [min max]
    return ((ctrl_sig + 1)*(max_sig - min_sig)/2) + min_sig
