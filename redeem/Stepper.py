#!/usr/bin/env python
"""
A Stepper Motor Driver class for Replicape.

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

import time
import logging
from Path import Path
from DAC import DAC
from ShiftRegister import ShiftRegister


class Stepper(object):

    all_steppers = list()
    
    def __init__(self, stepPin, dirPin, faultPin, dac_channel, shiftreg_nr, name, internalStepPin, internalDirPin):
        """ Init """
        self.dac_channel     = dac_channel  # Which channel on the dac is connected to this stepper
        self.stepPin         = stepPin
        self.dirPin          = dirPin
        self.faultPin        = faultPin
        self.name            = name
        self.enabled 	     = False	    
        self.in_use          = False        
        self.steps_pr_mm     = 1            
        self.microsteps      = 1.0          
        self.direction       = 1
        self.internalStepPin = (1 << internalStepPin)
        self.internalDirPin  = (1 << internalDirPin)

        # Set up the Shift register
        ShiftRegister.make()
        self.shift_reg = ShiftRegister.registers[shiftreg_nr]

    def get_state(self):
        """ Returns the current state """
        return self.state & 0xFF  # Return the state of the serial to parallel

    def update(self):
        """ Commits the changes	"""
        ShiftRegister.commit()  # Commit the serial to parallel

    # Higher level commands
    def set_steps_pr_mm(self, steps_pr_mm):
        """ Set the number of steps pr mm. """
        self.steps_pr_mm = steps_pr_mm
        self.mmPrStep = 1.0 / (steps_pr_mm * self.microsteps)
    
    def get_steps_pr_meter(self):
        """ Get the number of steps pr meter """
        return self.steps_pr_mm*self.microsteps * 1000.0

    def get_step_pin(self):
        """ The pin that steps, it looks like GPIO1_31 aso """
        return self.internalStepPin
    
    def get_dir_pin(self):
        """ Get the dir pin shifted into position """
        return self.internalDirPin

    def get_direction(self):
        return self.direction


"""
The bits in the shift register are as follows (Rev B1): 
Bit - name   - init val 
D0 = -		 = X (or servo enable)
D1 = CFG5    = 0 (Chopper blank time)
D2 = CFG4    = 0 (Choppper hysteresis)
D3 = CFG0    = 0 (Chopper off time)
D4 = CFG2    = 0 (microstepping)
D5 = CFG2-Z  = 0 (microstepping)
D6 = CFG1    = 0 (microstepping)
D7 = CFG1-Z  = 0 (microstepping)
"""

class Stepper_00B1(Stepper):

    def __init__(self, stepPin, dirPin, faultPin, dac_channel, shiftreg_nr, name, internalStepPin, internalDirPin):
        Stepper.__init__(self, stepPin, dirPin, faultPin, dac_channel, shiftreg_nr, name, internalStepPin, internalDirPin)
        self.dac    = DAC(dac_channel)
        self.state  = 0 # The initial state of shift register

    def set_microstepping(self, value, force_update=False):                
        """ Todo: Find an elegant way for this """
        self.microstepping = value
        self.microsteps  = 2**value     # 2^val
        self.state = int("0b"+bin(self.state)[2:].rjust(8, '0')[:4]+bin(value)[2:].rjust(3, '0')[::-1]+"0", 2)
        self.mmPrStep    = 1.0/(self.steps_pr_mm*self.microsteps)
        #logging.debug("Updated stepper "+self.name+" to microstepping "+str(self.microsteps))

        # update the Path class with new values
        stepper_num = Path.axis_to_index(self.name)
        Path.steps_pr_meter[stepper_num] = self.get_steps_pr_meter()

        self.shift_reg.remove_state(0xFF-0x01)
        self.shift_reg.set_state(self.state)

    def set_current_value(self, i_rms):
        """ Current chopping limit (This is the value you can change) """
        self.current_value = i_rms
        
        r_sense = 0.1020              # External resistors + internal
        sqrt_2 = 1.41421356237

        v_iref = 2.5*(i_rms/1.92)
        if(v_iref > 2.5):
            logging.warning("Current ref for stepper "+self.name+" above limit (2.5 V). Setting to 2.5 V")
            v_iref = 2.5
        logging.info("Setting votage to "+str(v_iref)+" for "+self.name)
        self.dac.set_voltage(v_iref)

    def set_disabled(self, force_update=False):
        pass

    def set_enabled(self, force_update=False):
        pass

    def set_decay(self, value):
        pass

"""
The bits in the shift register are as follows (Rev A4) :
Bit - name   - init val 
D0 = -		   = X
D1 = MODE2   = 0
D2 = MODE1   = 0
D3 = MODE0   = 0
D4 = nENABLE = 0  - Enabled
D5 = DECAY   = 0  - Slow decay 
D6 = nSLEEP  = 1  - Not sleeping 
D7 = nRESET  = 1  - Not in reset mode
"""

class Stepper_00A4(Stepper):
    revision    = "A4"
    SLEEP       = 6
    ENABLED     = 4
    RESET       = 7
    DECAY       = 5

    def __init__(self, stepPin, dirPin, faultPin, dac_channel, name, internalStepPin, internalDirPin):
        Stepper.__init__(self, stepPin, dirPin, faultPin, dac_channel, name, internalStepPin, internalDirPin)
        self.dacvalue 	     = 0x00   	    # The voltage value on the VREF		
        self.state           = (1<<Stepper.SLEEP)|(1<<Stepper.RESET)| (1<<Stepper.ENABLED) # The initial state of the inputs


    def set_enabled(self, force_update=False):
        """ Sets the Stepper enabled """
        if not self.enabled:
            self.state &= ~(1 << Stepper_00A4.ENABLED)
            self.enabled = True
        if force_update:
            self.update()

    def set_disabled(self, force_update=False):
        """ Sets the Stepper disabled """
        if self.enabled:
            self.state |= (1 << Stepper_00A4.ENABLED)
            self.enabled = False
        if force_update:
            self.update()

    def enable_sleepmode(self, force_update=False):
        """Logic high to enable device, logic low to enter
        low-power sleep mode. Internal pulldown."""
        self.state &= ~(1 << Stepper_00A4.SLEEP)
        if force_update:
            self.update()

    def disable_sleepmode(self, force_update=False):
        """ Disables sleepmode (awake) """
        self.state |= (1<<Stepper_00A4.SLEEP)
        if force_update:
            self.update()

    def reset(self, force_update=False):
        """nReset - Active-low reset input initializes the indexer
        logic and disables the H-bridge outputs.
        Internal pulldown."""
        self.state &= ~(1 << Stepper_00A4.RESET)
        self.update()
        time.sleep(0.001)
        self.state |= (1 << Stepper_00A4.RESET)
        self.update()
    
    def set_microstepping(self, value, force_update=False):        
        """ Microstepping (default = 0) 0 to 5 """        
        if not value in [0, 1, 2, 3, 4, 5]: # Full, half, 1/4, 1/8, 1/16, 1/32.
            logging.warning("Tried to set illegal microstepping value: {0} for stepper {1}".format(value, self.name))
            return
        self.microstepping = value
        self.microsteps  = 2**value     # 2^val
        # Keep bit 0, 4, 5, 6 intact but replace bit 1, 2, 3
        self.state = int("0b"+bin(self.state)[2:].rjust(8, '0')[:4]+bin(value)[2:].rjust(3, '0')+bin(self.state)[-1:], 2)
        #self.state = int("0b"+bin(self.state)[2:].rjust(8, '0')[:4]+bin(value)[2:].rjust(3, '0')+"0", 2)
        self.mmPrStep    = 1.0/(self.steps_pr_mm*self.microsteps)

        # update the Path class with new values
        stepper_num = Path.axis_to_index(self.name)
        Path.steps_pr_meter[stepper_num] = self.get_steps_pr_meter()

        if force_update:
            self.update()


    def set_current_value(self, iChop):
        """ Current chopping limit (This is the value you can change) """
        self.current_value = iChop
        if spi2_0 is None:
            return

        vRef = 3.3                   # Voltage reference on the DAC
        rSense = 0.1                 # Resistance for the
        vOut = iChop * 5.0 * rSense  # Calculated voltage out from the DAC

        self.dacval = int((vOut * 256.0) / vRef)
        byte1 = ((self.dacval & 0xF0) >> 4) | (self.dac_channel << 4)
        byte2 = (self.dacval & 0x0F) << 4
        spi2_0.writebytes([byte1, byte2])       # Update all channels

        # TODO: Change to only this channel (1<<dac_channel) ?
        spi2_0.writebytes([0xA0, 0xFF])

    def set_decay(self, value, force_update=False):
        """ Decay mode, look in the data sheet """
        self.decay = value
        self.state &= ~(1 << Stepper.DECAY)        # bit 5
        self.state |= (value << Stepper.DECAY)
        if force_update: 
            self.update()

"""
The bits in the shift register are as follows (Rev A3):
D0 = DECAY   = X
D1 = MODE0   = X
D2 = MODE1   = X
D3 = MODE2 	 = X
D4 = nRESET  = 1
D5 = nSLEEP  = 1
D6 = nENABLE = 0
D7 = -   		 = X
"""

class Stepper_00A3(Stepper_00A4):
    Stepper.revision = "A3"
    Stepper.ENABLED = 6
    Stepper.SLEEP = 5
    Stepper.RESET = 4
    Stepper.DECAY = 0

