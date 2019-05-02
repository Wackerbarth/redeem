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
        data = f.read(120)
        name = data[58:74].strip()
        if name == b"BB-BONE-REPLICAP":
          self.replicape_revision = data[38:42]
          self.replicape_data = data
          self.replicape_path = path
        elif name[:13] == b"BB-BONE-REACH":
          self.reach_revision = data[38:42]
          self.reach_data = data
          self.reach_path = path
    except IOError as e:
      pass


def aquire_printer_identifier(self):
  # Get the generated key or create one
  self.replicape_key = "".join(struct.unpack('20c', self.replicape_data[100:120]))
  logging.debug("Found Replicape key: '" + self.replicape_key + "'")
  if self.replicape_key == '\x00' * 20:
    logging.debug("Replicape key invalid")
    import random
    import string
    self.replicape_key = ''.join(
        random.SystemRandom().choice(string.ascii_uppercase + string.digits) for _ in range(20))
    self.replicape_data = self.replicape_data[:100] + self.replicape_key
    logging.debug("New Replicape key: '" + self.replicape_key + "'")
    #logging.debug("".join(struct.unpack('20c', self.new_replicape_data[100:120])))
    try:
      with open(self.replicape_path, "wb") as f:
        f.write(self.replicape_data[:120])
    except IOError as e:
      logging.warning("Unable to write new key to EEPROM")
