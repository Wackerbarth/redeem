import os
import sys


def system_cf(self, x):
  the_file = os.path.normpath(
      os.path.join(os.path.dirname(__file__), '..', '..', '..', 'configs', '{}.cfg'.format(x)))
  return the_file


def local_cf(self, x):
  the_file = os.path.normpath(os.path.join(sys.prefix, 'etc', '{}.cfg'.format(x)))
  return the_file


def testing_cf(self, x):
  the_file = os.path.normpath(os.path.join(self.testing_config_dir_path, '{}.cfg'.format(x)))
  return the_file
