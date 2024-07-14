########################
## SIMULATION CONFIGS ##
########################
### FIXED
N_STATE_VARS = 5
N_OUTPUT_VARS = 8
N_DEBUG_VARS = 7 # 3 Vphase, 3 EMF's, 1 star
N_SWITCHES = 6 # Upper / lower switches x 3

### TIME VARS
SIM_STEP = 1e-6
SIM_TIME = 1e-2 # 10 ms
### STATE VARS
# iu, iv, iw, angle, speed
X0 = [0, 1e-4, 0, 0, 0] # INITIAL CONDITION

#################
## BLDC PARAMS ##
#################

Inertia = 0.000007            # aka. 'J' in kg/(m^2)
tau_shaft = 0.006
B = Inertia/tau_shaft   # aka. 'B' in Nm/(rad/s)
Kv = 1./32.3*1000             # aka. motor constant in RPM/V
L = 0.00207                   # aka. Coil inductance in H
M = -0.00069                  # aka. Mutual inductance in H
R = 11.9                      # aka. Phase resistence in Ohm
VDC = 100.                    # aka. Supply voltage
NbPoles = 4.                  #
dvf = .0                      # aka. freewheeling diode forward voltage
T_fstatic = 1 # Static friction in Nm
T_load = 0   # Load torque (Write a fr)


#####################
## INVERTER PARAMS ##
#####################

V_DF = 1.3 # IRFZ44N: Diode forward voltage