################################################################################
# colors.py
#
# Description: Creates vizmo environment with obstacles colored according to
#  conflicts along a given path.
#
# Usage: TODO
################################################################################

import re


#TODO - add short descriptions to functions
def ListToString(l, number):
  if number <= 0:
    return ''

  string = l[0]
  for i in range(1, number):
    string += ' ' + str(l[i])
  return string

def GetPrefix(infile):
  return infile.read().partition('Passive')[0]

def ProcessComponent(string):
  nums = []
  while string.strip(' \n\t'):
    nums.append(re.findall('[+-]?\d+\.?\d*', string)[0])
    string = string.split(nums[-1], 1)[1]
  number = int(nums.pop(0).strip('.'))
  color = [nums.pop(0) for i in range(4)]
  pos = [nums.pop(0) for i in range(6)]
  return number, color, pos

def GetComponents(infile):
  parts = list(infile.read().split('Passive'))
  parts.pop(0)

  number, color, pos = [], [], []

  for part in parts:
    n, c, p = ProcessComponent(part)
    number.append(n)
    color.append(c)
    pos.append(p)

  return number, color, pos

# TODO - maybe allow a different color choice?
def AdjustColors(colors, labelfile):
  indices = [int(i) for i in labelfile.read().strip(' \n\t').split(' ')]
  for index in indices:
    colors[index] = ['1', '0', '0', '1']
  return colors

def WritePassiveComponent(number, color, pos, out):
  out.write('Passive\ncomponents/' + str(number) + '.obj -c(' + ListToString(color, 4) +
            ') -a(none) ' + ListToString(pos, 6) + '\n\n')

def WriteOutputFile(prefix, comps, out):
  out.write(prefix)
  for comp in comps:
    WritePassiveComponent(comp[0], comp[1], comp[2], out)

def Main():
  # Get arguments
  import argparse
  parser = argparse.ArgumentParser()

  # TODO - needs explanations about the formats of these files. Maybe in the "usage" part?
  parser.add_argument('-l', '--label-file', required=True, help='label file to read') #TODO should it be "path to..." ?
  parser.add_argument('-e', '--env-file', required=True, help='input env file name (vizmo version)') #TODO should it be "path to..." ?
  parser.add_argument('-o', '--output-file', required=True, help='output env file name')

  args = parser.parse_args()

  # Identify which components should have what colors.
  with open(str(args.env_file), 'r') as infile:
    prefix = GetPrefix(infile)
  with open(str(args.env_file), 'r') as infile:
    numbers, colors, poses = GetComponents(infile)

  # Read in label file.
  with open(str(args.label_file), 'r') as labelfile:
    colors = AdjustColors(colors, labelfile)

  comps = [(i, j, k) for i,j,k in zip(numbers, colors, poses)]

  # Print a new vizmo env.
  with open(str(args.output_file), 'w') as out:
    WriteOutputFile(prefix, comps, out)

if __name__ == '__main__':
  Main()

################################################################################
# END OF FILE
################################################################################

