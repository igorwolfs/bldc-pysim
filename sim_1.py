
#
# Open-BLDC pysim - Open BrushLess DC Motor Controller python simulator
# Copyright (C) 2011 by Antoine Drouin <poinix@gmail.com>
# Copyright (C) 2011 by Piotr Esden-Tempski <piotr@esden.net>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import matplotlib
import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate
from scipy.signal import decimate

import utils
import dyn_model  as dm
import control    as ctl
import my_plot    as mp
import config


def display_state_and_command(time, X, U):

    titles_state = ['$\\theta$', '$\omega$', '$i_u$', '$i_v$', '$i_w$']
    titles_cmd = ['$u_l$', '$u_h$', '$v_l$', '$v_h$', '$w_l$', '$w_h$']
    for i in range(0, 2):
        plt.subplot(6, 2, 2*i+1)
        plt.plot(time, utils.ANGLE_DEG2RAD * X[:,i], 'r', linewidth=3.0)
        plt.title(titles_state[i])
    for i in range(2, dm.sv_size):
        plt.subplot(6, 2, 2*i+1)
        plt.plot(time, X[:,i], 'r', linewidth=3.0)
        plt.title(titles_state[i])
    for i in range(0, 6):
        plt.subplot(6, 2, 2*i+2)
        plt.plot(time, U[:,i], 'r', linewidth=3.0)
        plt.title(titles_cmd[i])

def print_simulation_progress(count, steps):
        sim_perc_last = ((count-1)*100) / steps
        sim_perc = (count*100) / steps
        if (sim_perc_last != sim_perc):
            print(f"{sim_perc}")

def drop_it(a, factor):
	new = []
	for n, x in enumerate(a):
		if ((n % factor) == 0):
			new.append(x)
	return np.array(new)


def compress(a, factor):
	return drop_it(a, factor)


def main():

    # TIME VECTOR
    time = np.arange(0.0, config.SIM_TIME, config.SIM_STEP)
    
    # STATE VECTOR
    X = np.zeros((time.size, config.N_STATE_VARS))

    # OUTPUT VECTOR
    Y = np.zeros((time.size, config.N_OUTPUT_VARS))

    # INPUT VECTOR (which phases are excited)
    U = np.zeros((time.size, config.N_SWITCHES))

    # EMF's and phase voltages
    V_arr = np.zeros((time.size, config.N_DEBUG_VARS))

    X[0,:] = config.X0

    for i in range(1,time.size):
        if i==1:
            Uim2 = np.zeros(config.N_SWITCHES)
        else:
            Uim2 = U[i-2,:]

        # Get phase voltages, angle and rot-speed at t=i-1
        Y[i-1,:] = dm.output(X[i-1,:], Uim2)                  # get the output for the last step
        
        # 
        U[i-1,:] = ctl.run(0, Y[i-1,:], time[i-1])            # run the controller for the last step
        
        '''
        INPUT
        - dyn: function computing the derviative of y at t.
        - y: initial condition on y to integrate (vector)
        - t: sequence of time points to be solved (should be monotonically increasing or decreasing)
        - args: tuple of extra arguments to be passed ot the function
        RETURN:
        - y: array containing value of y for each desired time in t
        '''
        tmp = integrate.odeint(dm.dyn, X[i-1,:], [time[i-1], time[i]], args=(U[i-1,:],)) # integrate
        X[i,:] = tmp[1,:] # Copy integration output to the current step
        X[i, dm.sv_theta] = utils.angle_2pi( X[i, dm.sv_theta] ) # normalize the angle in the state
        tmp, V_arr[i,:] = dm.dyn_debug(X[i-1,:], time[i-1], U[i-1,:]) # get debug data
        print_simulation_progress(i, time.size)

    Y[-1,:] = Y[-2,:]
    U[-1,:] = U[-2,:]


    compress_factor = 3
    if compress_factor > 1:
        time = compress(time, compress_factor)
        Y = compress(Y, compress_factor)
        X = compress(X, compress_factor)
        U = compress(U, compress_factor)
        V_arr = compress(V_arr, compress_factor)

    mp.plot_output(time, Y, '-')
    plt.figure(figsize=(10.24, 5.12))
    display_state_and_command(time, X, U)

    plt.figure(figsize=(10.24, 5.12))
    mp.plot_debug(time, V_arr)

    plt.show()

if __name__ == "__main__":
    main()
