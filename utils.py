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

import math

### CONSTANTS
# velocity in rad/s to RPM
VEL_RADS2RPM = 60 / (2 * math.pi)

# Angle in degrees to rad
ANGLE_DEG2RAD = 180 / math.pi

# Velocity in rad/s to deg/m
DEGREES_2_RADS = VEL_RADS2RPM * 360

# Normalize angle between range of 0->2*pi
def angle_2pi(alpha):
    alpha_n = math.fmod(alpha, 2 * math.pi)
    if alpha_n < 0.:
        alpha_n = (2 * math.pi) + alpha_n
    return alpha_n


###############
## FUNCTIONS ##
###############

def trapezoid(angle):
    if 0. <= angle <= (math.pi / 6):
        return (1 / (math.pi /6 )) * angle
    elif (math.pi / 6) < angle <= (math.pi * (5 / 6)):
        return 1
    elif (math.pi * (5 / 6)) < angle <= (math.pi * (7 / 6)):
        return -((1/(math.pi/6.))* (angle - math.pi))
    elif (math.pi * (7 / 6)) < angle <= (math.pi * (11 /6)):
        return -1
    elif (math.pi * (11 / 6)) < angle <= (2 * math.pi):
        return (1 / (math.pi / 6)) * (angle - (2 * math.pi))
    else:
        raise ValueError("ERROR: angle out of bounds can not calculate bemf %d", phase_thetae)
