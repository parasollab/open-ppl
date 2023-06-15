################################################################################
# generate-problems.py
#
# Description: Creates and runs a problem for every component in the environment.
#  Then outputs the life-cycle metric if requested.
#
# Usage: Output depends on the input given. Always generates the problem output
#  files.
#  -Requires a config file. This is a json file which specifies:
#       -'seedfile-name': the path to the problem xml seed file.
#       -'xmlfile-name': the file prefix to use for the ouput problem xml files.
#       -'envfile-name': the file prefix to use for the output env files.
#       -'basefile-name': the file prefix to use for the pmpl generated output
#                         files (refered to as basefile in problem xmls).
#       -'robot-name': the file prefix for the given robots (robots must be
#                      named <robot-name>#.robot, where # ranges from 0 to the
#                      total number of robots - 1.
#       -'obj-base': the path to directory containing all .obj files.
#       -'seed': the random seed to be used for pmpl.
#       -'start-configs': ordered list of the starting configurations for each
#                         robot (i.e. first element is robot0.robot start)
#       -'goal-configs': ordered list of the goal configurations for each
#                        robot (i.e. first element is robot0.robot goal)
#       -'weights': ordered list of the component weights for life-cycle metric.
#  -If given a path to pmpl executable, runs the executable on the generated
#   problems.
#  -If -o arguement is given, computes and prints to console life-cycle metric.
################################################################################

import json
import os


#TODO - should we add the weights to the .env file?

# Writes an individual component to an output file.
def WritePassiveComponent(piece, color, pos, out):
  out.write('Passive\ncomponents/' + str(piece) + '.obj -c(' + color +
            ') -a(none) ' + pos + '\n\n')


# Writes the env file (either vizmo version, or pmpl version using vizmo flag).
def WriteEnvFile(piece, poses, config, vizmo):
  filename = config['envfile-name'] + str(piece)
  if vizmo:
    filename = filename + '-vizmo'
  filename = filename + '.env'

  bodycount = len(poses)
  if not vizmo:
    bodycount = bodycount - 1
  bodycount = str(bodycount)

  with open(filename, 'w') as envfile:
    envfile.write('Boundary Box [' + config['boundary-box'] + \
                  ']\n\nMultibodies\n' + bodycount + \
                  '\n\n')

    if vizmo:
      envfile.write('Active\n1\n' + config['obj-base'] + \
                    str(piece) + '.obj volumetric translational\nConnections\n0\n\n')

    for i, p in enumerate(poses):
      if i == piece:
        continue
      WritePassiveComponent(i, '0 1 0 .3', p, envfile)


# Writes the problem xml file.
def WriteXMLFile(piece, config):
  basefile = config['basefile-name'] + str(piece)
  envfile = config['envfile-name'] + str(piece) + '.env'
  robot = config['robot-name'] + str(piece) + '.robot'
  start = config['start-configs'][piece]
  goal = config['goal-configs'][piece]
  seed = config['seed']

  seedfile = config['seedfile-name']
  xmlfile = config['xmlfile-name'] + str(piece) + '.xml'

  os.system('cat ' + seedfile
    + '| sed "s|@@basefile-name@@|' + basefile + '|g"'
    + '| sed "s|@@envfile-name@@|' + envfile + '|g"'
    + '| sed "s|@@robot-name@@|' + robot + '|g"'
    + '| sed "s|@@start@@|' + start + '|g"'
    + '| sed "s|@@goal@@|' + goal + '|g"'
    + '| sed "s|@@seed@@|' + seed + '|g"'
    + '> ' + xmlfile)


def Main():

  # Get arguments
  import argparse
  parser = argparse.ArgumentParser()

  parser.add_argument('-c', '--config-file', required=True, help='path to file containing the configuration')
  parser.add_argument('-p', '--pmpl-exec', required=False, help='path to the pmpl executable')
  parser.add_argument('-o', '--compute-output', action='store_true', required=False, help='should the output be computed?')

  args = parser.parse_args()

  # Read in the config file.
  with open(str(args.config_file), 'r') as conffile:
    config = json.load(conffile)

  # Read in piece locations.
  poses = config['start-configs']

  for i, p in enumerate(poses):

    # Write out xml file.
    WriteXMLFile(i, config)

    # Write out env file.
    WriteEnvFile(i, poses, config, False)

    # Write out vizmo env file.
    WriteEnvFile(i, poses, config, True)

  # Run pmpl on each xml.
  if args.pmpl_exec is not None:

    import subprocess
    for i in range(len(poses)):
      subprocess.call([str(args.pmpl_exec), '-f', config['xmlfile-name'] + str(i) + '.xml'])

  conflicts = {}

  if args.compute_output:
    for i in range(len(poses)):
      with open(config['basefile-name'] + str(i) + '.' + config['seed'] + '.query.pc', 'r') as cf:
        conflicts[i] = [int(f) for f in cf.read().split()]

    # Get weights.
    wts = config['weights']

    #TODO - should we add the weights to the .env file?
    weights = {}
    for i, w in enumerate(wts):
      weights[i] = w

    # Compute metric.
    cost = 0.0
    for k in conflicts.keys():
      cost += weights[k] * len(conflicts[k])

    print('Found total removal cost: ', cost)
    return cost
  return None


if __name__ == '__main__':
  Main()

################################################################################
# END OF FILE
################################################################################

