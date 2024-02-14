from datetime import datetime
from itertools import zip_longest
import sys


def format_data(data):
  if len(data) <= 8:
    return '%s  %-8s' % (
      ' '.join(('--' if i is None else ('%02X' % i)) for i, _ in zip_longest(data, range(8))),
      bytes((j if 32 <= j <= 126 else 46) for j in data).decode('ascii', 'replace'),
    )
  else:
    data = data[:6]
    return '%s  ...   %-6s  ' % (
      ' '.join(('--' if i is None else ('%02X' % i)) for i, _ in zip_longest(data, range(6))),
      bytes((j if 32 <= j <= 126 else 46) for j in data[:6]).decode('ascii', 'replace'),
    )


def format_gv_modem_string(data):
  if len(data) != 8:
    return ''
  elif (0 <= data[7] < 0x80) or (0x90 <= data[7] < 0x100):
    return data.decode('ascii', 'replace')
  elif 0x80 <= data[7] < 0x88:
    return data[:data[7] - 0x80].decode('ascii', 'replace')
  return ''


def format_command(command):
  if not 0 <= command < 256: raise ValueError('command %s not between 0 and 255' % command)
  if command == 0x00: return '(0x00) SendReset'
  return ' '.join((
    '(0x%02X)' % command,
    '%X,' % (command >> 4),
    {0x0: 'RSVD 0',
     0x1: 'Flush ',
     0x2: 'RSVD 2',
     0x3: 'RSVD 3',
     0x4: 'RSVD 4',
     0x5: 'RSVD 5',
     0x6: 'RSVD 6',
     0x7: 'RSVD 7',
     0x8: 'Lstn 0',
     0x9: 'Lstn 1',
     0xA: 'Lstn 2',
     0xB: 'Lstn 3',
     0xC: 'Talk 0',
     0xD: 'Talk 1',
     0xE: 'Talk 2',
     0xF: 'Talk 3',
    }[command & 0xF],
  ))


def command_target(command):
  if not 0 <= command < 256: raise ValueError('command %s not between 0 and 255' % command)
  return command >> 4


def command_has_payload(command):
  if not 0 <= command < 256: raise ValueError('command %s not between 0 and 255' % command)
  return True if (command & 0x8) else False


class AdbAnalyzer:
  '''Represents an ADB analyzer.
  
  Note: serial_obj must have a timeout, though it should not be infinite.
  '''
  
  def __init__(self, serial_obj, device_filter=None):
    self.serial_obj = serial_obj
    self.device_filter = device_filter
  
  def run(self, gv_modem=False):
    last_str = None
    last_count = 0
    while True:
      command = self.serial_obj.read(1)
      if command == b'': continue
      command = command[0]
      command_str = format_command(command)
      if command_has_payload(command):
        data_len = self.serial_obj.read(1)
        if data_len == b'': raise ValueError('timeout waiting for payload length')
        data_len = data_len[0]
        data = self.serial_obj.read(data_len)
        if len(data) != data_len: raise ValueError('timeout waiting for payload')
        if gv_modem and (command & 0xF == 0):
          command_str = ' '.join((command_str, format_data(data), format_gv_modem_string(data)))
        else:
          command_str = ' '.join((command_str, format_data(data)))
      if self.device_filter is not None and command_target(command) != self.device_filter: continue
      if command_str == last_str:
        last_count += 1
      else:
        if last_str is not None and last_count > 1:
          sys.stdout.write(' x %d\n' % last_count)
        else:
          sys.stdout.write('\n')
        last_str = command_str
        last_count = 1
        sys.stdout.write(datetime.now().strftime('(%H:%M:%S) '))
        sys.stdout.write(command_str)
        sys.stdout.flush()
