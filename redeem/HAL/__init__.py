# -*- coding: utf-8 -*-
from __future__ import absolute_import


class Characteristics():
  from .transition.boards import identify_boards, aquire_printer_identifier

  reach_data = None
  reach_path = None
  reach_revision = None
  replicape_data = None
  replicape_key = None
  replicape_path = None
  replicape_revision = None


from .transition.devices import set_up_the_printer_devices
