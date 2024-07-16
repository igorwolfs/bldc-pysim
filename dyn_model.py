#-*- coding: utf-8 -*-
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
import utils
import config

# Components of the state vector
sv_theta  = 0      # angle of the rotor
sv_omega = 1       # angular speed of the rotor
sv_iu = 2          # phase u current
sv_iv = 3          # phase v current
sv_iw = 4          # phase w current

# Components of the command vector
iv_lu   = 0
iv_hu   = 1
iv_lv   = 2
iv_hv   = 3
iv_lw   = 4
iv_hw   = 5

# Components of the perturbation vector
pv_torque  = 0
pv_size = 2

# Components of the output vector
ov_iu      = 0
ov_iv      = 1
ov_iw      = 2
ov_vu      = 3
ov_vv      = 4
ov_vw      = 5
ov_theta   = 6
ov_omega   = 7
ov_size    = 8

# Phases and star vector designators
ph_U = 0
ph_V = 1
ph_W = 2
ph_star = 3
ph_size = 4

# Debug vector components
dv_eu = 0
dv_ev = 1
dv_ew = 2
dv_ph_U = 3
dv_ph_V = 4
dv_ph_W = 5
dv_ph_star = 6

#
# Calculate backemf at a given omega offset from the current rotor position
#
# Used to calculate the phase backemf aka. 'e'
#

# Calculate phase voltages
# Returns a vector of phase voltages in reference to the star point
def get_phase_voltages(X, U):
    emf = np.zeros(3, dtype=float)
    max_bemf = (utils.VEL_RADS2RPM * X[sv_omega]) / config.Kv
    for phase in range(3):
        # Mechanical -> Electrical angle (x poles / 2)
        angle = utils.angle_2pi((X[sv_theta] * (config.NbPoles / 2.)) + phase*2*math.pi/3)
        emf[phase] = max_bemf * utils.trapezoid(angle)

    X_currents = np.array([X[sv_iu], X[sv_iv], X[sv_iw]])

    ### Check which diodes are enabled
    #? How do we determine the neutral voltage when it is only calculable after knowing which switches are open or closed?
    #? Perhaps we can take the previous value of the voltage vm and assume it doesn't change that quickly?
    # D_arr = np.zeros(6)
    # for phase in range(3):
    #     if not (U[i*2+1] or U[i*2]):
    #         if (X[sv_iu+i] < 0) or (emf[i]+vm > config.VDC+config.V_DF):
    #             D_arr[i*2+1] = 1
    #         elif (X[sv_iu+i] < 0) or (emf[i]+vm+config.V_DF < 0):
    #             D_arr[i*2] = 1


    ### Check which phases are excited
    ph_en_arr = np.array([(U[iv_hu] == 1) or (U[iv_lu] == 1),
                        (U[iv_hv] == 1) or (U[iv_lv] == 1), 
                        (U[iv_hw] == 1) or (U[iv_lw] == 1)])

    ### Diodes: u, v, w, m
    V_arr = np.zeros(4)
    if (not np.any(ph_en_arr)):
        return V_arr # Voltage is zero when nothing is yet enabled
    
    # Case where 3/6 switches are switched (lower OR upper, can't be switched at the same time)
    if np.all(ph_en_arr):
        for i in range(3):
            if (U[i*2+1] == 1):
                V_arr[i] = config.VDC
            else:
                V_arr[i] = 0.

        #? What happens to the voltage drops over the resistor and inductor
        V_arr[3] = (np.sum(V_arr[:3]) - np.sum(emf)) / 3.
        return V_arr

    # Case where 2/6 switches are switched
    for i in range(3):
        j, k = (i+1)%3, (i+2)%3
        # The 2 enabled phases are equal to -V_DC or +V_DC depending on the upper / lower switch setting
        if ph_en_arr[j] and ph_en_arr[k]:
            for i_set in [j, k]:
                if (U[i_set*2+1] == 1):
                    V_arr[i_set] = config.VDC
                else:
                    V_arr[i_set] = 0.

            # The star voltage is the (2 array voltages - the 2 remaining emfs) / 2
            V_arr[3] = (V_arr[j] + V_arr[k] - emf[j] - emf[k]) / 2
            # The remaining phase voltage is the star voltage + the EMF
            V_arr[i] = emf[i] + V_arr[3]
            # if (X_currents[i] != 0):
                # raise ValueError("Current i is nonzero")
            # if np.any(V_arr[i] > config.VDC) or np.any(V_arr[i] < 0):
            #     raise ValueError("Freewheeling diodes conducting")
            '''
            Depending on the state of this 3rd voltage
            * > config.VDC + VDIODE: upper switch freewheeling diode will conduct
            * < -VDIODE: lower switch freewheeling diode will conduct
            '''

            return V_arr

    # Case where 1/6 switch is switched
    for i in range(3):
        if ph_en_arr[i]:
            if (U[i*2+1] == 1):
                V_arr[i] = config.VDC
            else:
                V_arr[i] = 0.
            V_arr[3] = V_arr[i] - emf[i]
            j, k = (i+1)%3, (i+2)%3
            V_arr[j] = V_arr[3] + emf[j]
            V_arr[k] = V_arr[3] + emf[k]
            
            # if (X_currents[j] != 0 or X_currents[k] != 0):
                # raise ValueError("Current i is nonzero")
            
            # if np.any(V_arr[i] > config.VDC) or np.any(V_arr[i] < 0):
            #     raise ValueError("Freewheeling diodes conducting")
            return V_arr
    
    raise ValueError("ERR: None of the switches enabled")


def output(X, U):

    V = get_phase_voltages(X, U)
    Y = [X[sv_iu], X[sv_iv], X[sv_iw],
         V[ph_U], V[ph_V], V[ph_W],
         X[sv_theta], X[sv_omega]]
    return Y

#
# Dynamic model
#
# X state, t time, U input, W perturbation
#

def dyn(X, t, U):
    Xd, Xdebug = dyn_debug(X, t, U)
    return Xd


# Dynamic model with debug vector
def dyn_debug(X, t, U):
    emf = np.zeros(3, dtype=float)

    max_bemf = (utils.VEL_RADS2RPM * X[sv_omega]) / config.Kv

    for phase in range(3):
        angle = utils.angle_2pi((X[sv_theta] * (config.NbPoles / 2.)) + phase*2*math.pi/3)
        # BACK-EMF IS OF TRAPEZOIDAL SHAPE
        emf[phase] = max_bemf * utils.trapezoid(angle)

    # Energy equation: torque =  (EM-energy) / omega
    etorque = np.dot(emf, X[sv_iu:sv_iw+1]) / X[sv_omega]

    # Mechanical torque (subtracting config.B and load torque)
    mtorque = ((etorque * (config.NbPoles / 2)) - (config.B * X[sv_omega]) - config.T_load)

    # Include static friction
    if ((mtorque > 0) and (mtorque <= config.T_fstatic)):
        mtorque = 0
    elif (mtorque >= config.T_fstatic):
        mtorque = mtorque - config.T_fstatic
    elif ((mtorque < 0) and (mtorque >= (-config.T_fstatic))):
	    mtorque = 0
    elif (mtorque <= (-config.T_fstatic)):
        mtorque = mtorque + config.T_fstatic

    # Acceleration of the rotor
    omega_dot = mtorque / config.Inertia

    V = get_phase_voltages(X, U)

    Xd = [X[sv_omega],
          omega_dot,
          (V[ph_U] - (config.R * X[sv_iu]) - emf[0] - V[ph_star]) / (config.L - config.M), #diu/dt
          (V[ph_V] - (config.R * X[sv_iv]) - emf[1] - V[ph_star]) / (config.L - config.M), #div/dt
          (V[ph_W] - (config.R * X[sv_iw]) - emf[2] - V[ph_star]) / (config.L - config.M)  #diw/dt
    ]

    Xdebug = [
        emf[0],
        emf[1],
        emf[2],
        V[ph_U],
        V[ph_V],
        V[ph_W],
        V[ph_star]
        ]

    return Xd, Xdebug
