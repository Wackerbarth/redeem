# -*- coding: utf-8 -*-
from __future__ import absolute_import

import logging
import os
import sys
from configobj import ConfigObj
from redeem.CascadingConfigParser import CascadingConfigParser


class Configuration(CascadingConfigParser):
  def __init__(self, printer, *args, **kwargs):
    _characteristics = printer.characteristics
    if 'config_dir_path' in kwargs:
      _characteristics.testing_config_dir_path = kwargs['config_dir_path']
      _characteristics.system_cf = _characteristics.testing_cf
      _characteristics.local_cf = _characteristics.testing_cf
    default_cfg_path = _characteristics.system_cf('default')
    if not os.path.exists(default_cfg_path):
      logging.error(default_cfg_path + " does not exist, this file is required for operation")
      sys.exit()    # maybe use something more graceful?
    configuration_files_list = [default_cfg_path]

    _selected_printer = 'printer'
    if '-selected' in args:
      try:
        _p = ConfigObj('/usr/local/etc/Select_A_Service.conf')['Services']['redeem']['printer']
        _selected_printer = _p
      except:
        pass
    if '--printer' in kwargs:
      _selected_printer = kwargs['--printer']

    printer_cfg_path = None
    _cfg_path = _characteristics.system_cf(_selected_printer)
    if os.path.exists(_cfg_path):
      printer_cfg_path = _cfg_path
      configuration_files_list.append(printer_cfg_path)

    _cfg_path = _characteristics.local_cf(_selected_printer)
    if os.path.exists(_cfg_path):
      printer_cfg_path = _cfg_path
      configuration_files_list.append(_cfg_path)

    if printer_cfg_path is None:
      logging.warning(printer_cfg_path + " does not exist, proceed with caution")

    local_cfg_path = _characteristics.local_cf('local')
    if not os.path.exists(local_cfg_path):
      logging.info(local_cfg_path + " does not exist, Creating one")
      os.mknod(local_cfg_path)
      os.chmod(local_cfg_path, 0o666)
    configuration_files_list.append(local_cfg_path)

    # Parse the configuration files.
    CascadingConfigParser.__init__(self, configuration_files_list)

    # Check the local and printer files
    if not printer_cfg_path is None:
      self.check(printer_cfg_path)
    self.check(local_cfg_path)
