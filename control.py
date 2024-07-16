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

import numpy as np
import math

import dyn_model  as dm
import utils
import config

PWM_freq = 16000.
PWM_cycle_time = (1./PWM_freq)
PWM_duty = 0.6
PWM_duty_time = PWM_cycle_time * PWM_duty

debug = False

'''  Switching angle based on actual mechanical (And thus electrical / (nPoles/2)) angle measurement
The switching pattern below uses the electrical angle to set the right switches at the right time.
The on-time depends on the duty cycle and the pwm-frequency. 
The startup is not handled, apparently the power is large enough to make the motor simply start running.

'''
def run_hpwm_l_on_bipol(Y, t):
    
    # Get the electrical angle
    elec_angle = utils.angle_2pi(Y[dm.ov_theta] * config.NbPoles/2)

    # Switch states
    U = np.zeros(config.N_SWITCHES)

    step = "none"

    # Switching pattern based on the "encoder"
    upper_b_arr = (math.pi/6.)*np.array([1., 3., 5., 7., 9., 11., 12.], dtype=np.float64)
    upper_sw_arr = np.array([dm.iv_hw, dm.iv_hu, dm.iv_hu, dm.iv_hv, dm.iv_hv, dm.iv_hw, dm.iv_hw])
    lower_b_arr = upper_b_arr - np.pi/6 # first and last step in steps of pi/6
    lower_b_arr[1:6] -= np.pi/6 # Everything between 1 and 6 in steps of pi/3
    lower_sw_arr = np.array([dm.iv_lv, dm.iv_lw, dm.iv_lw, dm.iv_lu, dm.iv_lu, dm.iv_lv, dm.iv_lv])

    # print(lower_b_arr)
    for step in range(len(upper_b_arr)+1):
        #print(f"lower: {lower_b_arr[step]} < {elec_angle} < {upper_b_arr[step]}")
        if lower_b_arr[step] <= np.float(elec_angle) <= upper_b_arr[step]: # second half of step 1
            # print(f"Step: {step}, angle: {elec_angle}")
            U[lower_sw_arr[step]] = 1
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                U[upper_sw_arr[step]] = 1
            break

    #? Why are v and w phases here switched in order to make it work?
    #? Something to do with switching rotation direction
    hv = U[dm.iv_hv]
    lv = U[dm.iv_lv]
    hw = U[dm.iv_hw]
    lw = U[dm.iv_lw]

    U[dm.iv_hv] = hw
    U[dm.iv_lv] = lw
    U[dm.iv_hw] = hv
    U[dm.iv_lw] = lv

    if debug:
        print(f'time {t} step {step} eangle {utils.ANGLE_DEG2RAD * elec_angle} switches {U}')

    return U

###
### Sensorless control
###
# Enable 2 phases at a time
# Get the non-excited phase
# Get the phase voltage there
# If this phase voltage crosses zero compared to the neutral point: change switching sequence

def sensorless(Y, t):
    
    # Get the electrical angle
    elec_angle = utils.angle_2pi(Y[dm.ov_theta] * config.NbPoles/2)

    # Switch states
    U = np.zeros(config.N_SWITCHES)

    step = "none"

    # Switching pattern based on the "encoder"
    upper_b_arr = (math.pi/6.)*np.array([1., 3., 5., 7., 9., 11., 12.], dtype=np.float64)
    upper_sw_arr = np.array([dm.iv_hw, dm.iv_hu, dm.iv_hu, dm.iv_hv, dm.iv_hv, dm.iv_hw, dm.iv_hw])
    lower_b_arr = upper_b_arr - np.pi/6 # first and last step in steps of pi/6
    lower_b_arr[1:6] -= np.pi/6 # Everything between 1 and 6 in steps of pi/3
    lower_sw_arr = np.array([dm.iv_lv, dm.iv_lw, dm.iv_lw, dm.iv_lu, dm.iv_lu, dm.iv_lv, dm.iv_lv])

    # print(lower_b_arr)
    for step in range(len(upper_b_arr)+1):
        #print(f"lower: {lower_b_arr[step]} < {elec_angle} < {upper_b_arr[step]}")
        if lower_b_arr[step] <= np.float(elec_angle) <= upper_b_arr[step]: # second half of step 1
            # print(f"Step: {step}, angle: {elec_angle}")
            U[lower_sw_arr[step]] = 1
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                U[upper_sw_arr[step]] = 1
            break

    hv = U[dm.iv_hv]
    lv = U[dm.iv_lv]
    hw = U[dm.iv_hw]
    lw = U[dm.iv_lw]

    U[dm.iv_hv] = hw
    U[dm.iv_lv] = lw
    U[dm.iv_hw] = hv
    U[dm.iv_lw] = lv

    if debug:
        print(f'time {t} step {step} eangle {utils.ANGLE_DEG2RAD * elec_angle} switches {U}')

    return U

#
# Sp setpoint, Y output
#
def run(Y, t):
    #return run_hpwm_l_on(Sp, Y, t)
    return run_hpwm_l_on_bipol(Y, t)
