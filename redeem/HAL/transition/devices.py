# -*- coding: utf-8 -*-
from __future__ import absolute_import

import glob
import logging
import os
from six import PY2, iteritems
if PY2:
  import Queue as queue
else:
  import queue

from redeem.Alarm import Alarm, AlarmExecutor
from redeem.CascadingConfigParser import CascadingConfigParser
from redeem.ColdEnd import ColdEnd
from redeem.Cooler import Cooler
from redeem.Delta import Delta
from redeem.Enable import Enable
from redeem.EndStop import EndStop
from redeem.Ethernet import Ethernet
from redeem.Extruder import Extruder, HBP
from redeem.Fan import Fan
from redeem.FilamentSensor import *
from redeem.Gcode import Gcode
from redeem.GCodeProcessor import GCodeProcessor
from redeem.HAL import Characteristics
from redeem.IOManager import IOManager
from redeem.Key_pin import Key_pin, Key_pin_listener
from redeem.Mosfet import Mosfet
from redeem.Path import Path
from redeem.PathPlanner import PathPlanner
from redeem.Pipe import Pipe
from redeem.PluginsController import PluginsController
from redeem.Printer import Printer
from redeem.PruFirmware import PruFirmware
from redeem.PWM import PWM
from redeem.RotaryEncoder import *
from redeem.Servo import Servo
from redeem.Stepper import *
from redeem.StepperWatchdog import StepperWatchdog
from redeem.TemperatureSensor import *
from redeem.USB import USB
from redeem.Watchdog import Watchdog


def set_up_the_printer_devices(printer):
  """
  Here we create the interface objects which represent the hardware components
  and we initialize them based on the values in the configuration files
  """
  configuration = printer.config
  _revision = printer.characteristics.replicape_revision
  if _revision in ["00A4", "0A4A", "00A3"]:
    PWM.set_frequency(100)
  elif _revision in ["00B1", "00B2", "00B3", "0B3A"]:
    PWM.set_frequency(1000)

  # Init the Watchdog timer
  printer.watchdog = Watchdog()

  # Enable PWM and steppers
  printer.enable = Enable("P9_41")
  printer.enable.set_disabled()

  printer.NUM_EXTRUDERS = configuration.getint('Steppers', 'number_of_extruders')

  # Init the Paths
  printer.axis_config = configuration.getint('Geometry', 'axis_config')

  printer.endstop_io_manager = IOManager()
  # Init the end stops
  EndStop.inputdev = configuration.get("Endstops", "inputdev")
  # Set up key listener
  Key_pin.listener = Key_pin_listener(EndStop.inputdev, printer.endstop_io_manager)

  homing_only_endstops = configuration.get('Endstops', 'homing_only_endstops')

  for es in ["Z2", "Y2", "X2", "Z1", "Y1",
             "X1"]:    # Order matches end stop inversion mask in Firmware
    pin = configuration.get("Endstops", "pin_" + es)
    keycode = configuration.getint("Endstops", "keycode_" + es)
    invert = configuration.getboolean("Endstops", "invert_" + es)
    printer.end_stops[es] = EndStop(printer, pin, keycode, es, invert)
    printer.end_stops[es].stops = configuration.get('Endstops', 'end_stop_' + es + '_stops')

  # activate all the endstops
  printer.set_active_endstops()

  # Init the 5 Stepper motors (step, dir, fault, DAC channel, name)
  Stepper.printer = printer
  if _revision == "00A3":
    printer.steppers["X"] = Stepper_00A3("GPIO0_27", "GPIO1_29", "GPIO2_4", 0, "X")
    printer.steppers["Y"] = Stepper_00A3("GPIO1_12", "GPIO0_22", "GPIO2_5", 1, "Y")
    printer.steppers["Z"] = Stepper_00A3("GPIO0_23", "GPIO0_26", "GPIO0_15", 2, "Z")
    printer.steppers["E"] = Stepper_00A3("GPIO1_28", "GPIO1_15", "GPIO2_1", 3, "E")
    printer.steppers["H"] = Stepper_00A3("GPIO1_13", "GPIO1_14", "GPIO2_3", 4, "H")
  elif _revision == "00B1":
    printer.steppers["X"] = Stepper_00B1("GPIO0_27", "GPIO1_29", "GPIO2_4", 11, 0, "X")
    printer.steppers["Y"] = Stepper_00B1("GPIO1_12", "GPIO0_22", "GPIO2_5", 12, 1, "Y")
    printer.steppers["Z"] = Stepper_00B1("GPIO0_23", "GPIO0_26", "GPIO0_15", 13, 2, "Z")
    printer.steppers["E"] = Stepper_00B1("GPIO1_28", "GPIO1_15", "GPIO2_1", 14, 3, "E")
    printer.steppers["H"] = Stepper_00B1("GPIO1_13", "GPIO1_14", "GPIO2_3", 15, 4, "H")
  elif _revision == "00B2":
    printer.steppers["X"] = Stepper_00B2("GPIO0_27", "GPIO1_29", "GPIO2_4", 11, 0, "X")
    printer.steppers["Y"] = Stepper_00B2("GPIO1_12", "GPIO0_22", "GPIO2_5", 12, 1, "Y")
    printer.steppers["Z"] = Stepper_00B2("GPIO0_23", "GPIO0_26", "GPIO0_15", 13, 2, "Z")
    printer.steppers["E"] = Stepper_00B2("GPIO1_28", "GPIO1_15", "GPIO2_1", 14, 3, "E")
    printer.steppers["H"] = Stepper_00B2("GPIO1_13", "GPIO1_14", "GPIO2_3", 15, 4, "H")
  elif _revision in ["00B3", "0B3A"]:
    printer.steppers["X"] = Stepper_00B3("GPIO0_27", "GPIO1_29", 90, 11, 0, "X")
    printer.steppers["Y"] = Stepper_00B3("GPIO1_12", "GPIO0_22", 91, 12, 1, "Y")
    printer.steppers["Z"] = Stepper_00B3("GPIO0_23", "GPIO0_26", 92, 13, 2, "Z")
    printer.steppers["E"] = Stepper_00B3("GPIO1_28", "GPIO1_15", 93, 14, 3, "E")
    printer.steppers["H"] = Stepper_00B3("GPIO1_13", "GPIO1_14", 94, 15, 4, "H")
  elif _revision in ["00A4", "0A4A"]:
    printer.steppers["X"] = Stepper_00A4("GPIO0_27", "GPIO1_29", "GPIO2_4", 0, 0, "X")
    printer.steppers["Y"] = Stepper_00A4("GPIO1_12", "GPIO0_22", "GPIO2_5", 1, 1, "Y")
    printer.steppers["Z"] = Stepper_00A4("GPIO0_23", "GPIO0_26", "GPIO0_15", 2, 2, "Z")
    printer.steppers["E"] = Stepper_00A4("GPIO1_28", "GPIO1_15", "GPIO2_1", 3, 3, "E")
    printer.steppers["H"] = Stepper_00A4("GPIO1_13", "GPIO1_14", "GPIO2_3", 4, 4, "H")
  # Init Reach steppers, if present.
  if printer.characteristics.reach_revision == "00A0":
    printer.steppers["A"] = Stepper_reach_00A4("GPIO2_2", "GPIO1_18", "GPIO0_14", 5, 5, "A")
    printer.steppers["B"] = Stepper_reach_00A4("GPIO1_16", "GPIO0_5", "GPIO0_14", 6, 6, "B")
    printer.steppers["C"] = Stepper_reach_00A4("GPIO0_3", "GPIO3_19", "GPIO0_14", 7, 7, "C")
  elif printer.characteristics.reach_revision == "00B0":
    printer.steppers["A"] = Stepper_reach_00B0("GPIO1_16", "GPIO0_5", "GPIO0_3", 5, 5, "A")
    printer.steppers["B"] = Stepper_reach_00B0("GPIO2_2", "GPIO0_14", "GPIO0_3", 6, 6, "B")

  # Enable the steppers and set the current, steps pr mm and
  # microstepping
  for name, stepper in iteritems(printer.steppers):
    stepper.in_use = configuration.getboolean('Steppers', 'in_use_' + name)
    stepper.direction = configuration.getint('Steppers', 'direction_' + name)
    stepper.has_endstop = configuration.getboolean('Endstops', 'has_' + name)
    stepper.set_current_value(configuration.getfloat('Steppers', 'current_' + name))
    stepper.set_steps_pr_mm(configuration.getfloat('Steppers', 'steps_pr_mm_' + name))
    stepper.set_microstepping(configuration.getint('Steppers', 'microstepping_' + name))
    stepper.set_decay(configuration.getint("Steppers", "slow_decay_" + name))
    # Add soft end stops
    printer.soft_min[Printer.axis_to_index(name)] = configuration.getfloat(
        'Endstops', 'soft_end_stop_min_' + name)
    printer.soft_max[Printer.axis_to_index(name)] = configuration.getfloat(
        'Endstops', 'soft_end_stop_max_' + name)
    slave = configuration.get('Steppers', 'slave_' + name)
    if slave:
      printer.add_slave(name, slave)
      logging.debug("Axis " + name + " has slave " + slave)

  # Delta printer setup
  if printer.axis_config == Printer.AXIS_CONFIG_DELTA:
    opts = ["L", "r", "A_radial", "B_radial", "C_radial", "A_angular", "B_angular", "C_angular"]
    for opt in opts:
      Delta.__dict__[opt] = configuration.getfloat('Delta', opt)

  # Discover and add all DS18B20 cold ends.
  paths = glob.glob("/sys/bus/w1/devices/28-*/w1_slave")
  logging.debug("Found cold ends: " + str(paths))
  for i, path in enumerate(paths):
    printer.cold_ends.append(ColdEnd(path, "ds18b20-" + str(i)))
    logging.info("Found Cold end " + str(i) + " on " + path)

  # Make Mosfets, temperature sensors and extruders
  heaters = ["E", "H", "HBP"]
  if printer.characteristics.reach_revision:
    heaters.extend(["A", "B", "C"])
  for e in heaters:
    # Mosfets
    channel = configuration.getint("Heaters", "mosfet_" + e)
    printer.mosfets[e] = Mosfet(channel)
    # Thermistors
    adc = configuration.get("Heaters", "path_adc_" + e)
    if not configuration.has_option("Heaters", "sensor_" + e):
      sensor = configuration.get("Heaters", "temp_chart_" + e)
      logging.warning("Deprecated config option temp_chart_" + e + " use sensor_" + e + " instead.")
    else:
      sensor = configuration.get("Heaters", "sensor_" + e)
    printer.thermistors[e] = TemperatureSensor(adc, 'MOSFET ' + e, sensor)
    printer.thermistors[e].printer = printer

    # Extruders
    onoff = configuration.getboolean('Heaters', 'onoff_' + e)
    prefix = configuration.get('Heaters', 'prefix_' + e)
    if e != "HBP":
      printer.heaters[e] = Extruder(printer.steppers[e], printer.thermistors[e], printer.mosfets[e],
                                    e, onoff)
    else:
      printer.heaters[e] = HBP(printer.thermistors[e], printer.mosfets[e], onoff)
    printer.heaters[e].prefix = prefix
    printer.heaters[e].Kp = configuration.getfloat('Heaters', 'pid_Kp_' + e)
    printer.heaters[e].Ti = configuration.getfloat('Heaters', 'pid_Ti_' + e)
    printer.heaters[e].Td = configuration.getfloat('Heaters', 'pid_Td_' + e)

    # Min/max settings
    printer.heaters[e].min_temp = configuration.getfloat('Heaters', 'min_temp_' + e)
    printer.heaters[e].max_temp = configuration.getfloat('Heaters', 'max_temp_' + e)
    printer.heaters[e].max_temp_rise = configuration.getfloat('Heaters', 'max_rise_temp_' + e)
    printer.heaters[e].max_temp_fall = configuration.getfloat('Heaters', 'max_fall_temp_' + e)
    printer.heaters[e].max_power = configuration.getfloat('Heaters', 'max_power_' + e)

  # Init the three fans. Argument is PWM channel number
  printer.fans = []
  if _revision == "00A3":
    printer.fans.append(Fan(0))
    printer.fans.append(Fan(1))
    printer.fans.append(Fan(2))
  elif _revision == "0A4A":
    printer.fans.append(Fan(8))
    printer.fans.append(Fan(9))
    printer.fans.append(Fan(10))
  elif _revision in ["00B1", "00B2", "00B3", "0B3A"]:
    printer.fans.append(Fan(7))
    printer.fans.append(Fan(8))
    printer.fans.append(Fan(9))
    printer.fans.append(Fan(10))
  if printer.characteristics.reach_revision == "00A0":
    printer.fans.append(Fan(14))
    printer.fans.append(Fan(15))
    printer.fans.append(Fan(7))

  # Set default value for all fans
  for i, f in enumerate(printer.fans):
    f.set_value(configuration.getfloat('Fans', "default-fan-{}-value".format(i)))

  # Init the servos
  printer.servos = []
  servo_nr = 0
  while (configuration.has_option("Servos", "servo_" + str(servo_nr) + "_enable")):
    if configuration.getboolean("Servos", "servo_" + str(servo_nr) + "_enable"):
      channel = configuration.get("Servos", "servo_" + str(servo_nr) + "_channel")
      pulse_min = configuration.getfloat("Servos", "servo_" + str(servo_nr) + "_pulse_min")
      pulse_max = configuration.getfloat("Servos", "servo_" + str(servo_nr) + "_pulse_max")
      angle_min = configuration.getfloat("Servos", "servo_" + str(servo_nr) + "_angle_min")
      angle_max = configuration.getfloat("Servos", "servo_" + str(servo_nr) + "_angle_max")
      angle_init = configuration.getfloat("Servos", "servo_" + str(servo_nr) + "_angle_init")
      s = Servo(channel, pulse_min, pulse_max, angle_min, angle_max, angle_init)
      printer.servos.append(s)
      logging.info("Added servo " + str(servo_nr))
    servo_nr += 1

  # Connect thermistors to fans
  for t, therm in iteritems(printer.heaters):
    for f, fan in enumerate(printer.fans):
      if not configuration.has_option('Cold-ends', "connect-therm-{}-fan-{}".format(t, f)):
        continue
      if configuration.getboolean('Cold-ends', "connect-therm-{}-fan-{}".format(t, f)):
        c = Cooler(therm, fan, "Cooler-{}-{}".format(t, f), True)    # Use ON/OFF on these.
        c.ok_range = 4
        opt_temp = "therm-{}-fan-{}-target_temp".format(t, f)
        if configuration.has_option('Cold-ends', opt_temp):
          target_temp = configuration.getfloat('Cold-ends', opt_temp)
        else:
          target_temp = 60
        c.set_target_temperature(target_temp)
        max_speed = "therm-{}-fan-{}-max_speed".format(t, f)
        if configuration.has_option('Cold-ends', max_speed):
          target_speed = configuration.getfloat('Cold-ends', max_speed)
        else:
          target_speed = 1.0
        c.set_max_speed(target_speed)
        c.enable()
        printer.coolers.append(c)
        logging.info("Cooler connects therm {} with fan {}".format(t, f))

  # Connect fans to M106
  printer.controlled_fans = []
  for i, fan in enumerate(printer.fans):
    if not configuration.has_option('Cold-ends', "add-fan-{}-to-M106".format(i)):
      continue
    if configuration.getboolean('Cold-ends', "add-fan-{}-to-M106".format(i)):
      printer.controlled_fans.append(printer.fans[i])
      logging.info("Added fan {} to M106/M107".format(i))

  # Connect the colds to fans
  for ce, cold_end in enumerate(printer.cold_ends):
    for f, fan in enumerate(printer.fans):
      option = "connect-ds18b20-{}-fan-{}".format(ce, f)
      if configuration.has_option('Cold-ends', option):
        if configuration.getboolean('Cold-ends', option):
          c = Cooler(cold_end, fan, "Cooler-ds18b20-{}-{}".format(ce, f), False)
          c.ok_range = 4
          opt_temp = "cooler_{}_target_temp".format(ce)
          if configuration.has_option('Cold-ends', opt_temp):
            target_temp = configuration.getfloat('Cold-ends', opt_temp)
          else:
            target_temp = 60
          c.set_target_temperature(target_temp)
          c.enable()
          printer.coolers.append(c)
          logging.info("Cooler connects temp sensor ds18b20 {} with fan {}".format(ce, f))

  # Init encoders
  printer.filament_sensors = []
  printer.rotary_encoders = []
  for ex in ["E", "H", "A", "B", "C"]:
    if not configuration.has_option('Rotary-encoders', "enable-{}".format(ex)):
      continue
    if configuration.getboolean("Rotary-encoders", "enable-{}".format(ex)):
      logging.debug("Rotary encoder {} enabled".format(ex))
      event = configuration.get("Rotary-encoders", "event-{}".format(ex))
      cpr = configuration.getint("Rotary-encoders", "cpr-{}".format(ex))
      diameter = configuration.getfloat("Rotary-encoders", "diameter-{}".format(ex))
      r = RotaryEncoder(event, cpr, diameter)
      printer.rotary_encoders.append(r)
      # Append as Filament Sensor
      ext_nr = Printer.axis_to_index(ex) - 3
      sensor = FilamentSensor(ex, r, ext_nr, printer)
      alarm_level = configuration.getfloat("Filament-sensors", "alarm-level-{}".format(ex))
      logging.debug("Alarm level" + str(alarm_level))
      sensor.alarm_level = alarm_level
      printer.filament_sensors.append(sensor)

  # Make a queue of commands
  printer.commands = queue.Queue(10)

  # Make a queue of commands that should not be buffered
  printer.unbuffered_commands = queue.Queue(10)

  # Bed compensation matrix
  printer.matrix_bed_comp = printer.load_bed_compensation_matrix()
  logging.debug("Loaded bed compensation matrix: \n" + str(printer.matrix_bed_comp))

  for axis in printer.steppers.keys():
    i = Printer.axis_to_index(axis)
    printer.max_speeds[i] = configuration.getfloat('Planner', 'max_speed_' + axis.lower())
    printer.max_speed_jumps[i] = configuration.getfloat('Planner', 'max_jerk_' + axis.lower())
    printer.home_speed[i] = configuration.getfloat('Homing', 'home_speed_' + axis.lower())
    printer.home_backoff_speed[i] = configuration.getfloat('Homing',
                                                           'home_backoff_speed_' + axis.lower())
    printer.home_backoff_offset[i] = configuration.getfloat('Homing',
                                                            'home_backoff_offset_' + axis.lower())
    printer.backlash_compensation[i] = configuration.getfloat('Steppers',
                                                              'backlash_' + axis.lower())

  printer.e_axis_active = configuration.getboolean('Planner', 'e_axis_active')

  dirname = os.path.dirname(os.path.realpath(os.path.join(__file__, '..', '..')))

  # Create the firmware compiler..
  pru_firmware = PruFirmware(dirname + "/firmware/firmware_runtime.c",
                             dirname + "/firmware/firmware_runtime.bin",
                             dirname + "/firmware/firmware_endstops.c",
                             dirname + "/firmware/firmware_endstops.bin", printer, "/usr/bin/clpru",
                             dirname + "/firmware/AM335x_PRU.cmd", dirname + "/firmware/image.cmd")

  printer.move_cache_size = configuration.getfloat('Planner', 'move_cache_size')
  printer.print_move_buffer_wait = configuration.getfloat('Planner', 'print_move_buffer_wait')
  printer.max_buffered_move_time = configuration.getfloat('Planner', 'max_buffered_move_time')

  printer.processor = GCodeProcessor(printer)
  printer.plugins = PluginsController(printer)

  # Path planner
  travel_default = False
  center_default = False
  home_default = False

  # Setting acceleration before PathPlanner init
  for axis in printer.steppers.keys():
    printer.acceleration[Printer.axis_to_index(axis)] = configuration.getfloat(
        'Planner', 'acceleration_' + axis.lower())

  printer.path_planner = PathPlanner(printer, pru_firmware)
  for axis in printer.steppers.keys():
    i = Printer.axis_to_index(axis)

    # Sometimes soft_end_stop aren't defined to be at the exact hardware boundary.
    # Adding 100mm for searching buffer.
    if configuration.has_option('Geometry', 'travel_' + axis.lower()):
      printer.path_planner.travel_length[axis] = configuration.getfloat(
          'Geometry', 'travel_' + axis.lower())
    else:
      printer.path_planner.travel_length[axis] = (printer.soft_max[i] - printer.soft_min[i]) + .1
      if axis in ['X', 'Y', 'Z']:
        travel_default = True

    if configuration.has_option('Geometry', 'offset_' + axis.lower()):
      printer.path_planner.center_offset[axis] = configuration.getfloat(
          'Geometry', 'offset_' + axis.lower())
    else:
      printer.path_planner.center_offset[axis] = (printer.soft_min[i] if printer.home_speed[i] > 0
                                                  else printer.soft_max[i])
      if axis in ['X', 'Y', 'Z']:
        center_default = True

    if configuration.has_option('Homing', 'home_' + axis.lower()):
      printer.path_planner.home_pos[axis] = configuration.getfloat('Homing', 'home_' + axis.lower())
    else:
      printer.path_planner.home_pos[axis] = printer.path_planner.center_offset[axis]
      if axis in ['X', 'Y', 'Z']:
        home_default = True

  if printer.axis_config == Printer.AXIS_CONFIG_DELTA:
    if travel_default:
      logging.warning(
          "Axis travel (travel_*) set by soft limits, manual setup is recommended for a delta")
    if center_default:
      logging.warning(
          "Axis offsets (offset_*) set by soft limits, manual setup is recommended for a delta")
    if home_default:
      logging.warning("Home position (home_*) set by soft limits or offset_*")
      logging.info("Home position will be recalculated...")

      # convert home_pos to effector space
      Az = printer.path_planner.home_pos['X']
      Bz = printer.path_planner.home_pos['Y']
      Cz = printer.path_planner.home_pos['Z']

      delta_bot = printer.path_planner.native_planner.delta_bot

      z_offset = delta_bot.verticalOffset(Az, Bz, Cz)    # vertical offset
      xyz = delta_bot.deltaToWorld(Az, Bz, Cz)    # effector position

      # The default home_pos, provided above, is based on effector space
      # coordinates for carriage positions. We need to transform these to
      # get where the effector actually is.
      xyz[2] += z_offset
      for i, a in enumerate(['X', 'Y', 'Z']):
        printer.path_planner.home_pos[a] = xyz[i]

      logging.info("Home position = %s" % str(printer.path_planner.home_pos))

  # Read end stop value again now that PRU is running
  for _, es in iteritems(printer.end_stops):
    es.read_value()

  # Enable Stepper timeout
  timeout = configuration.getint('Steppers', 'timeout_seconds')
  printer.swd = StepperWatchdog(printer, timeout)
  if configuration.getboolean('Steppers', 'use_timeout'):
    printer.swd.start()

  # Set up communication channels
  printer.comms_io_manager = IOManager()
  printer.comms["USB"] = USB(printer, printer.comms_io_manager)
  printer.comms["Eth"] = Ethernet(printer)
  printer.comms["octoprint"] = Pipe(printer, "octoprint", printer.comms_io_manager)
  printer.comms["toggle"] = Pipe(printer, "toggle", printer.comms_io_manager)
  printer.comms["testing"] = Pipe(printer, "testing", printer.comms_io_manager)
  printer.comms["testing_noret"] = Pipe(printer, "testing_noret", printer.comms_io_manager)
  # Does not send "ok"
  printer.comms["testing_noret"].send_response = False
