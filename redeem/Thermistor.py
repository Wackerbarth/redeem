#!/usr/bin/env python
"""
A Thermistor class for Replicape.

Author: Elias Bakken
email: elias(dot)bakken(at)gmail(dot)com
Website: http://www.thing-printer.com
License: GNU GPL v3: http://www.gnu.org/copyleft/gpl.html

 Redeem is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Redeem is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Redeem.  If not, see <http://www.gnu.org/licenses/>.
"""

import numpy as np
import logging
from threading import Lock

# Import the temp chart. 
from temp_chart import *


class Thermistor:
    """ Represents a thermistor """

    def __init__(self, pin, name, chart_name):
        """ Init """
        self.pin = pin
        self.name = name
        self.temp_table = np.array(temp_chart[chart_name]).transpose()

    def get_temperature(self):
        """ Return the temperature in degrees celsius """
        with open(self.pin, "r") as file:
            try:
                voltage = (float(file.read().rstrip()) / 4096) * 1.8
                res_val = self.voltage_to_resistance(voltage)  # Convert to resistance
                return self.resistance_to_degrees(res_val) # Convert to degrees
            except IOError as e:
                logging.error("Unable to get ADC value ({0}): {1}".format(e.errno, e.strerror))
                return -1.0

    def resistance_to_degrees(self, resistor_val):
        """ Return the temperature nearest to the resistor value """
        idx = (np.abs(self.temp_table[1] - resistor_val)).argmin()
        return self.temp_table[0][idx]

    def voltage_to_resistance(self, v_sense):
        """ Convert the voltage to a resistance value """
        if v_sense == 0:
            return 10000000.0
        return 4700.0 / ((1.8 / v_sense) - 1.0)
