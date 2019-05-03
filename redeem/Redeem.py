#!/usr/bin/env python
"""
Redeem main program. This should run on the BeagleBone.

Author: Elias Bakken
email: elias(at)iagent(dot)no
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

import logging
import logging.handlers
import os
import signal
import sys
from six import PY2, iteritems
from threading import Thread
from threading import enumerate as enumerate_threads
if PY2:
  import Queue as queue
  from Queue import Empty as EmptyQueueException
else:
  import queue
  from queue import Empty as EmptyQueueException

from .Alarm import Alarm, AlarmExecutor
from .CascadingConfigParser import CascadingConfigParser
from .Gcode import Gcode
from .HAL import Characteristics, set_up_the_printer_devices
from .Key_pin import Key_pin
from .Path import Path
from .Printer import Printer
from .Stepper import Stepper
from .StepperWatchdog import StepperWatchdog
from .Watchdog import Watchdog

from .configuration import standard_configuration
# Global vars
printer = None
TheControllerIsRunning = False

# Default logging level is set to debug
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
    datefmt='%m-%d %H:%M')


class TopLevelController:
  def __init__(self, *args, **kargs):
    """
    config_location: provide the location to look for config files.
     - default is installed directory
     - allows for running in a local directory when debugging
    """
    from .__init__ import __version__
    logging.info("Redeem initializing {}".format(__version__))
    global printer
    printer = Printer()
    self.printer = printer
    Path.printer = printer
    Gcode.printer = printer

    # Set up and Test the alarm framework
    Alarm.printer = self.printer
    Alarm.executor = AlarmExecutor()
    alarm = Alarm(Alarm.ALARM_TEST, "Alarm framework operational")

    # check for config files
    file_path = os.path.join(config_location, "default.cfg")
    if not os.path.exists(file_path):
      logging.error(file_path + " does not exist, this file is required for operation")
      sys.exit()    # maybe use something more graceful?

    local_path = os.path.join(config_location, "local.cfg")
    if not os.path.exists(local_path):
      logging.info(local_path + " does not exist, Creating one")
      os.mknod(local_path)
      os.chmod(local_path, 0o666)

    # Provide storage for hardware characteristics
    printer.characteristics = Characteristics()

    # Parse the config files.
    ###### standard_configuration()
    printer.config = CascadingConfigParser([
        os.path.join(config_location, 'default.cfg'),
        os.path.join(config_location, 'printer.cfg'),
        os.path.join(config_location, 'local.cfg')
    ])

    # Check the local and printer files
    printer_path = os.path.join(config_location, "printer.cfg")
    if os.path.exists(printer_path):
      printer.config.check(printer_path)
    printer.config.check(os.path.join(config_location, 'local.cfg'))

    # Get the revision and loglevel from the Config file
    level = self.printer.config.getint('System', 'loglevel')
    if level > 0:
      logging.getLogger().setLevel(level)

    # Set up additional logging, if present:
    if self.printer.config.getboolean('System', 'log_to_file'):
      logfile = self.printer.config.get('System', 'logfile')
      formatter = '%(asctime)s %(name)-12s %(levelname)-8s %(message)s'
      printer.redeem_logging_handler = logging.handlers.RotatingFileHandler(
          logfile, maxBytes=2 * 1024 * 1024)
      printer.redeem_logging_handler.setFormatter(logging.Formatter(formatter))
      printer.redeem_logging_handler.setLevel(level)
      logging.getLogger().addHandler(printer.redeem_logging_handler)
      logging.info("-- Logfile configured --")

    # Find out which capes are connected
    printer.characteristics.identify_boards()
    printer.characteristics.aquire_printer_identifier()

    # We set it to 5 axis by default
    Printer.NUM_AXES = 5
    if self.printer.characteristics.reach_revision == "00A0":
      Printer.NUM_AXES = 8
    elif self.printer.characteristics.reach_revision == "00B0":
      Printer.NUM_AXES = 7
    # Build the objects that represent the hardware
    set_up_the_printer_devices(printer)

  def start(self):
    """ Start the processes """
    global TheControllerIsRunning
    TheControllerIsRunning = True
    # Start the two processes
    p0 = Thread(target=self.loop, args=(self.printer.commands, "buffered"), name="p0")
    p1 = Thread(target=self.loop, args=(self.printer.unbuffered_commands, "unbuffered"), name="p1")
    p0.daemon = True
    p1.daemon = True

    p0.start()
    p1.start()

    Alarm.executor.start()
    Key_pin.listener.start()

    if self.printer.config.getboolean('Watchdog', 'enable_watchdog'):
      self.printer.watchdog.start()

    self.printer.enable.set_enabled()

    # Signal everything ready
    logging.info("Redeem ready")

  def loop(self, the_queue, name):
    """ When a new gcode comes in, execute it """
    try:
      while TheControllerIsRunning:
        try:
          gcode = the_queue.get(block=True, timeout=1)
        except EmptyQueueException:
          continue
        logging.debug("Executing " + gcode.code() + " from " + name + " " + gcode.message)
        self._execute(gcode)
        self.printer.reply(gcode)
        the_queue.task_done()
        logging.debug("Completed " + gcode.code() + " from " + name + " " + gcode.message)
    except Exception:
      logging.exception("Exception in {} loop: ".format(name))

  def exit(self):
    global TheControllerIsRunning
    if not TheControllerIsRunning:
      return
    TheControllerIsRunning = False
    logging.info("Redeem Shutting Down")
    printer.path_planner.wait_until_done()
    printer.path_planner.force_exit()

    # Stops plugins
    self.printer.plugins.exit()

    for name, stepper in iteritems(self.printer.steppers):
      stepper.set_disabled()
    Stepper.commit()

    for name, heater in iteritems(self.printer.heaters):
      logging.debug("closing " + name)
      heater.disable()

    for name, endstop in iteritems(self.printer.end_stops):
      logging.debug("terminating " + name)
      endstop.stop()

    for name, comm in iteritems(self.printer.comms):
      logging.debug("closing " + name)
      comm.close()

    self.printer.comms_io_manager.stop()

    self.printer.enable.set_disabled()
    self.printer.swd.stop()
    Alarm.executor.stop()
    Key_pin.listener.stop()
    self.printer.endstop_io_manager.stop()
    self.printer.watchdog.stop()
    self.printer.enable.set_disabled()

    # list all threads that are still running
    # note: some of these may be daemons
    for t in enumerate_threads():
      if t.name != "MainThread":
        logging.debug("Thread " + t.name + " is still running")

  def _execute(self, g):
    """ Execute a G-code """
    if g.message == "ok" or g.code() == "ok" or g.code() == "No-Gcode":
      g.set_answer(None)
      return
    if g.is_info_command():
      desc = self.printer.processor.get_long_description(g)
      self.printer.send_message(g.prot, desc)
    else:
      self.printer.processor.execute(g)

  def _synchronize(self, g):
    """ Synchronized execution of a G-code """
    self.printer.processor.synchronize(g)


def main(*args, **kwargs):
  # Create Top Level Controller
  the_controller = TopLevelController(*args, **kwargs)

  def signal_handler(signal, frame):
    logging.warning("Received signal: {}, terminating".format(signal))
    the_controller.exit()

  def signal_logger(signal, frame):
    logging.warning("Received signal: {}, ignoring".format(signal))

  # Register signal handler to allow interrupt with CTRL-C
  signal.signal(signal.SIGINT, signal_handler)
  signal.signal(signal.SIGTERM, signal_handler)

  # Register signal handler to ignore other signals
  signal.signal(signal.SIGHUP, signal_logger)

  # Launch Redeem
  the_controller.start()

  logging.info("Startup complete - main thread sleeping")

  # Wait for end of process signal
  global TheControllerIsRunning
  while TheControllerIsRunning:
    signal.pause()

  logging.info("Redeem Terminated")


def profile(*args, **kwargs):
  import yappi
  yappi.start()
  main(*args, **kwargs)
  yappi.get_func_stats().print_all()


if __name__ == '__main__':
  if len(sys.argv) > 1 and sys.argv[1] == "profile":
    profile()
  else:
    main()
