# -*- coding: utf-8 -*-
from __future__ import absolute_import

import glob
import logging
import struct


def identify_boards(self):
  """ Read the name and revision of each cape on the BeagleBone """
  paths = glob.glob("/sys/bus/i2c/devices/[1-2]-005[4-7]/*/nvmem")
  paths.extend(glob.glob("/sys/bus/i2c/devices/[1-2]-005[4-7]/nvmem/at24-[1-4]/nvmem"))
  #paths.append(glob.glob("/sys/bus/i2c/devices/[1-2]-005[4-7]/eeprom"))
  for i, path in enumerate(paths):
    try:
      with open(path, "rb") as f:
        data = bytearray(f.read(120))
        name = data[58:74].decode('utf-8').strip()
        _revision = data[38:42].decode('utf-8')
        if name == "BB-BONE-REPLICAP":
          self.replicape_revision = _revision
          self.replicape_data = data
          self.replicape_path = path
          logging.info("Found Replicape rev. {}".format(self.replicape_revision))
        elif name[:13] == "BB-BONE-REACH":
          self.reach_revision = _revision
          self.reach_data = data
          self.reach_path = path
          logging.info("Found Reach rev. {}".format(self.reach_revision))
    except IOError as e:
      pass
    # Proceed as if the board was present
    # There were some B3A boards that were distributed without being pre-programmed
    # TODO -- Is there any reason to keep this hack?
  if self.replicape_revision == None:
    logging.warning("No Replicape found")
    self.replicape_revision = "0B3A"


def aquire_printer_identifier(self):
  # Get the generated key or create one
  _key = bytes(self.replicape_data[100:120])
  if _key == bytes(20):
    logging.debug("Uninitialized Replicape key")
    import random
    import string
    _key = bytes(
        random.SystemRandom().choice(string.ascii_uppercase + string.digits) for _ in range(20))
    self.replicape_data[100:120] = _key
    logging.debug("New Replicape key: '{}'".format(_key.decode('utf-8')))
    #logging.debug("".join(struct.unpack('20c', self.new_replicape_data[100:120])))
    try:
      with open(self.replicape_path, "wb") as f:
        f.write(self.replicape_data[:120])
    except IOError as e:
      logging.warning("Unable to write new key to EEPROM")
  self.replicape_key = _key.decode('utf-8')
  logging.debug("Found Replicape key: '{}'".format(self.replicape_key))
