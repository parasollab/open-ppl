#!/bin/python
'''
Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
component of the Texas A&M University System.
All rights reserved.
The information and source code contained herein is the exclusive
property of TEES and may not be disclosed, examined or reproduced
in whole or in part without explicit written authorization from TEES.
'''

import sys
import os.path
import getopt
import re
import json
from create_flows import generate_flows_files, indent
from StringIO import StringIO

class Statement:
  '''
  Represents one line in a skeleton composition and consists of a variable name,
  a skeleton, and the input data
  '''

  def __init__(self, output_variable_name, skeleton, input_list, description):
    self.output_variable_name = output_variable_name
    self.skeleton = skeleton
    self.input_list = input_list
    self.description = description

  def to_string(self):
    return self.output_variable_name + ' <= ' + self.skeleton + \
           str(self.input_list)

class CompositionParser:
  '''
  This compiler converts the input composition to a set of Statements which
  can be converted to JSON files using the toJSON method. The generated
  JSON method can be used to generate a customized flows struct for
  the given composition with the help of the create_flows module.

  The input file should have the following format:

  input  = [v0 v1 v2]
  v3     = skeleton1  [v0]
  v4     = skeleton2  [v3 v0]
  output = [v4]

  The input and output are keywords and should be used as described above.
  However, the names used for any other variable can be arbitrary and can
  even contain parenthesis for cases such as make_repeat_view(v1). Each line
  should have the following format:
  v = skeleton [v...]
  In which skeleton can be empty for input and output statements.
  '''
  input_list_regex = '\[([\w\(\)\s]+)\]'
  statement_regex  = '(\w+)\s*=\s*(.*)\s*' + input_list_regex + '(\s*#\s*)?(.*)'
  empty_line_regex = '^$'
  regexes = [statement_regex, empty_line_regex]

  def __create_node__(self, line):
    '''
    Creates a Statement instance from a parsed line.
    '''
    g = re.match(self.statement_regex, line)
    comment = g.group(5) if g.groups > 4 and g.group(5) else ''
    return Statement(g.group(1), g.group(2), g.group(3).split(), comment)

  def __portmaps__(self, statements):
    '''
    Generates a portmap dictionary from the statements which can be used
    to generate the JSON output.
    '''
    ports_dic = {}
    if statements:
      statement_index = 0
      for statement in statements:
        var_name = statement.output_variable_name
        # first we find the sources of variables
        if var_name == 'input':
          for v in statement.input_list:
            # create a new entry
            ports_dic[v] = \
              {'source' : {'port'  : 'inflow',
                           'index' : str(statement.input_list.index(v))},
               'targets' : []}
        elif var_name != 'output':
          ports_dic[var_name] = \
            {'source'  : {'port' : str(statement_index)},
             'targets' : [],
             'description' : statement.description}
        # now we define the targets of each variable. For each statement except
        # the input we will look at the input_list
        if var_name != 'input':
          v_index = 0
          for v in statement.input_list:
            port_name = str(statement_index) if var_name != 'output' \
                        else 'outflow'
            target = {'port'   : port_name,
                      'index'  : str(v_index)}
            ports_dic[v]['targets'].append(target)
            v_index += 1
        if var_name not in ['input', 'output']:
          statement_index += 1
    return ports_dic

  def validate(self, input_filename):
    '''
    Checks if the given input filename has the correct syntax.
    '''
    i = 0
    passed = True

    input_file = open(input_filename, 'r')
    for line in input_file:
      if not any(map(lambda x : re.match(x, line) , self.regexes)):
        print 'Syntax error in line {} : {}'.format(i, line.replace('\n', ''))
        passed = False
      i += 1
    input_file.close()
    return passed

  def parse(self, input_filename):
    '''
    Parses the composition input file to a list of Statements.
    '''
    input_file = open(input_filename, 'r')
    statements = [self.__create_node__(line.strip())
                  for line in input_file if line.strip()]
    input_file.close()
    return statements

  def toJSON(self, statements):
    '''
    Generates a customized flows JSON file from the given set of statements.
    '''
    json_dic = {}
    ports_dic = self.__portmaps__(statements)
    statements_count = 0
    for statement in statements:
      var_name = statement.output_variable_name
      if var_name == 'input':
        json_dic['inflow'] = {}
        json_dic['inflow']['output'] = []
        for v in statement.input_list:
          for i in ports_dic[v]['targets']:
            json_dic['inflow']['output'].append(
              {'to' : i['port'], 'filter' : {'index' : i['index']}}
            )
      elif var_name == 'output':
        json_dic['outflow'] = {}
        json_dic['outflow']['input'] = []
        for v in statement.input_list:
          json_dic['outflow']['input'].append(
            {'to' : ports_dic[v]['source']['port']})
      else:
        port_id = ports_dic[var_name]['source']['port']
        json_dic[port_id] = {}
        json_dic[port_id]['input'] = []
        for v in statement.input_list:
          item = {}
          src = ports_dic[v]['source']
          item['to'] = src['port']
          if 'index' in src:
            item['filter'] = {'index' : src['index']}
          json_dic[port_id]['input'].append(item)

        if ports_dic[var_name]['description'] != '':
          json_dic[port_id]['description'] = ports_dic[var_name]['description']

        json_dic[port_id]['output'] = []
        for v in ports_dic[var_name]['targets']:
          target = v['port']
          item = {}
          item['to'] = target
          if 'index' in v:
            item['filter'] = {'index' : v['index']}
          json_dic[port_id]['output'].append(item)

        statements_count += 1

    json_dic['size'] = statements_count
    return json_dic

def print_composition(statements, flows_name):
  '''
  Prints out the composition without the inputs which can be used in the
  C++ file.
  '''
  sts = [s for s in statements if s.output_variable_name != 'input' and\
                                  s.output_variable_name != 'output']

  print 'compose<flows::' + flows_name + '>('
  print indent(''.join('%s' %',\n'.join(map(lambda x : x.skeleton, sts)))) + ')'

def generate_composition(input_filename, flows_name):
  '''
  Read a skeleton composition file, and create the flows.hpp file for that,
  in addition prints the c++ composition on the screen.
  '''
  parser = CompositionParser()
  statements = parser.parse(input_filename)

  if statements:
    print 'The composition code is:'
    print_composition(statements, flows_name)
    print '\nGenerating files:'
    data = parser.toJSON(statements)
    generate_flows_files(data, flows_name)

def help():
  print './create_composition.py -i <input-composition-file>'\
        ' -f <flows-name>'

def main():
  try:
    opts, args = getopt.getopt(
                   sys.argv[1:], 'hf:i:',
                   ['help', 'flows-name', 'composition-file'])
  except getopt.error, msg:
    help();
    sys.exit(2)
  input_filename = ''
  flows_name = ''
  for o, a in opts:
    if o in ('-h', '--help'):
      help()
      sys.exit(0)
    if o in ('-f', '--flows-name'):
      flows_name = a
    if o in ('-i', '--composition-file'):
      input_filename = a


  if flows_name == '' or input_filename == '':
    help()
    sys.exit(2)

  f = None
  if os.path.isfile(input_filename):
    generate_composition(input_filename, flows_name)
  else:
    print "The input file does not exist"
    sys.exit(2)

if __name__ == '__main__':
  main()
