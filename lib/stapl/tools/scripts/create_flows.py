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
from StringIO import StringIO
from pprint import pprint

def indent(s, tabs=2):
  indent_text = ''
  for i in range(0, tabs):
    indent_text += ' '
  return re.sub('^', indent_text, s, flags=re.M)


def stapl_license_text():
  '''
  Adds stapl license to the generated file.
  '''
  s = '/*\n'\
    '// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES),'\
    ' a\n// component of the Texas A&M University System.\n'\
    '// All rights reserved.\n'\
    '// The information and source code contained herein is the exclusive\n'\
    '// property of TEES and may not be disclosed, examined or reproduced\n'\
    '// in whole or in part without explicit written authorization from TEES'\
    '.\n*/\n\n'

  return s

class stapl_skeletons_wrappers(object):
  '''
  Adds the hash defines for the generated file.
  '''
  def __init__(self, f):
    self.f = f

  def __call__(self, arg1, arg2):
    s = stapl_license_text()
    s += '#ifndef STAPL_' + arg2.upper() + '_FLOWS_HPP\n'\
         '#define STAPL_' + arg2.upper() + '_FLOWS_HPP\n\n'\
         '#include <utility>\n'\
         '#include <stapl/skeletons/flows/flow_helpers.hpp>\n\n'

    s += 'namespace flows {\n\n'\
         'using namespace stapl::skeletons::flows;\n\n'
    s += self.f(arg1, arg2)
    s += '\n\n} // namespace flows\n'
    s += '\n\n#endif // STAPL_' + arg2.upper() + '_FLOWS_HPP\n'
    return s

class doxygen_documentation(object):
  '''
  Creates the documentation for the flow using the description and layout
  given in the flow description file
  '''

  def __init__(self, f):
    self.f = f

  def __call__(self, data, name):
    s = ''
    s += ''\
    '//////////////////////////////////////////////////////////////////////\n'\
    '/// @brief '

    if 'description' in data:
      for elem in data['description']:
        if elem != '':
          s += ' '
        s += elem + '\n///'
      s += '\n'
    else:
      s += 'AUTOMATICALLY GENERATED DOCUMENTATION\n'
    s += ''\
    '///\n'\
    '/// @dot\n'
    graphviz_data = ''

    graphviz_data += ''\
    'digraph ' + name + ' {\n'\
    '  rankdir = LR;\n'\
    '  node [shape=record];\n'\
    '  in [label="Inflow", color="white"];\n'\
    '  out [label="Outflow", color="white"];\n'\
    '  subgraph cluster_' + name.lower() + ' {\n'
    size = len(data.keys())
    for key in data:
      if key not in['outflow', 'inflow', 'size', 'description']:
        graphviz_data += ''\
        '  a' + key + '[label=<'
        if 'description' in data[key]:
          graphviz_data += data[key]['description']
        else:
          graphviz_data += 'P<sub>' + key + '</sub>'

        graphviz_data += '>]\n'

    graphviz_data += ''\
        '  label=<Compose<sub>[' + name.lower() + ']</sub>>;\n'
    graphviz_data += '  }\n'
    #This is where the actual piping is dumped

    for flow_desc,flow_value in data.iteritems():
      # we print information for everything connected to the input of each
      # element
      if flow_desc not in ['inflow', 'size', 'description']:
        if 'input' in flow_value:
          for connection in flow_value['input']:
            node_name = ''
            if connection['to'] == 'inflow':
              node_name = 'in'
            else:
              node_name = 'a' + connection['to']

            graphviz_data +='  ' + node_name + ' -> '
            if flow_desc != 'outflow':
              graphviz_data += 'a' + flow_desc
            else:
              graphviz_data += 'out'

            if 'filter' in connection:
              graphviz_data += '[label="'

              if connection['filter'] in ['front', 'back']:
                graphviz_data += connection['filter']
              else:
                graphviz_data += 'get<' + str(connection['filter']['index']) +\
                                 '>'
              graphviz_data += '(' + node_name + ')"]'
            graphviz_data += ';\n'
        else:
          print 'Error in your file, no "inflow" is defined for item ' + \
                flow_desc
    graphviz_data += '}'
    filename = name + '.dot'
    dotfile = open(filename, 'w')
    dotfile.write(graphviz_data + '\n')

    print 'Generated ' + filename
    s += re.sub('^', '///  ', graphviz_data, flags=re.M)
    s += ''\
    '\n/// @enddot\n'\
    '//////////////////////////////////////////////////////////////////////\n'

    s += self.f(data, name)
    return s


def create_flow_tuple_type(elements, func):
  '''
  Creates a result_of::flow_bundle with the given list of elements by applying
  the given function on each element
  '''
  s = ''
  if len(elements) > 1:
    s += 'flows::result_of::concat<\n'
    s += indent(''.join('%s' %',\n'.join(map(func, elements))))
    s += '\n>'
  else:
    s += func(elements[0])
  return s

def create_flow_tuple(elements, func):
  '''
  Creates a flow_bundle with the given list of elements by applying
  the given func on each element.
  '''
  s = ''
  if len(elements) > 1:
    s += 'concat(\n'
    s += indent(''.join('%s' %',\n'.join(map(func, elements))))
    s += '\n)'
  elif len(elements) == 1:
    s += func(elements[0])
  else:
    s += 'dontcare_flow()'
  return s

def create_method(templates, signature, return_statement,\
                  first_line='', return_type='auto'):
  '''
  Creates a method with a given signature and return types.
  '''
  s = first_line                                         +\
      templates                                   + '\n' +\
      return_type                                 + '\n' +\
      signature                                   + '\n'

  if return_type == 'auto':
    s += '-> decltype('                           + '\n' +\
         indent(return_statement)
    s += '\n)\n'
  s += '{'                                        + '\n' +\
       indent('return ' + return_statement + ';') + '\n' +\
       '}'                                        + '\n'
  return s

def create_filter_call(flow_desc, flows):
  '''
  Creates a filter over the given flow.
  '''
  if 'filter' in flow_desc:
    f = flow_desc['filter']
    filter_name = ('tuple_ops::' + f) if f in ['front', 'back']\
                    else 'std::get<' + str(f['index']) + '>'
    return 'stapl::make_tuple(\n  ' + filter_name + '(' + flows + '))'
  else:
    return flows

def create_filter_call_type(flow_desc, flows):
  '''
  Creates a filter type over the given flow type.
  '''
  if 'filter' in flow_desc:
    f = flow_desc['filter']
    filter_name = 'tuple_ops::result_of::' + f + '<\n  ' + flows + '\n>::type' \
                  if f in ['front', 'back']\
                  else 'stapl::tuple_element <\n' + \
                       '  ' + str(f['index']) + ',\n' + \
                       '  ' + flows + '>::type\n'
    return 'stapl::tuple<typename ' + filter_name + '>'
  else:
    return flows

def create_inner_out_port_type(flow_id):
  '''
  Creates the type of an out_port_type for each stage based on the description
  given in the json flow description.
  '''
  return 'typename\n'\
         '  stapl::tuple_element<' + flow_id + ', skeletons_t>::type::\n'\
         '    template out_port_type<in_flow_' + flow_id + '_type>::type'

def create_out_port_type(flow_id):
  '''
  Creates the type of an out_port_type for each stage based on the description
  given in the json flow description.
  '''
  return 'In'\
          if flow_id == 'inflow'\
          else 'typename inner_ports_types<In>::out_port_' + flow_id + '_type'

def create_in_flow_from_out_ports(output_list):
  '''
  Creates the type of an in_flow based on the out_flows to which it is
  connected.
  '''
  return create_flow_tuple_type(output_list,
           lambda flow :
             create_filter_call_type(flow,
               'In' if flow['to'] == 'inflow'
               else 'out_port_' + flow['to'] + '_type'))

def create_in_flows(input_list):
  '''
  Creates the in_flow return instruction for one stage of the compose.
  '''
  return create_flow_tuple(input_list,
           lambda flow :
             create_filter_call(flow,
               'in' if flow['to'] == 'inflow'
                else
                'm_compose.template get_out_port<\n'         +
                '  typename inner_ports_types<In>::in_flow_' +
                flow['to'] + '_type , ' + flow['to'] + '\n>(lid_offset)'))

def create_in_flows_type(output_list):
  '''
  Creates the type of an in_flow_type for each stage based on the description
  given in the json flow description.
  '''
  return 'dontcare_flow' if not output_list\
    else\
      create_flow_tuple_type(output_list,
        lambda flow :
          create_filter_call_type(flow,
            'Out' if flow['to'] == 'outflow'
            else 'typename stapl::tuple_element<' + flow['to'] +
                 ', skeletons_t>::type::in_port_type'))

def create_out_flows(output_list):
  '''
  Creates the out_flow return instruction for one stage of the compose.
  '''
  s = ''
  return 'dontcare_flow()' if output_list is None\
  else\
    create_flow_tuple(output_list,
      lambda flow :
        create_filter_call(flow,
        'out' if flow['to'] == 'outflow'
        else 'm_compose.template get_in_port<' +
             flow['to'] + '>(lid_offset)'))

def create_in_flow_method(index, input_list, size):
  '''
  Creates the in_flow method for one stage of the compose.
  '''
  is_last = (index == str(size-1))
  s = 'in_flow(In const& in, std::size_t lid_offset,'         + '\n' + \
      '        std::integral_constant<int, ' + index + '>,'   + '\n' + \
      '        std::integral_constant<bool, ' + str(is_last).lower() + \
      '> /*!is_last*/)'
  return_type = create_flow_tuple_type(input_list,
                  lambda flow :
                    create_filter_call_type(
                      flow, create_out_port_type(flow['to'])))

  return create_method(templates       ='template <typename In>',
                       signature       =s,
                       return_statement=create_in_flows(input_list),
                       return_type     =return_type)


def create_out_flow_method(index, output_list, size):
  '''
  Creates the out_flow method for one stage of the compose.
  '''
  is_last = (index == str(size-1))
  s = 'out_flow(Out const& out, std::size_t lid_offset,'     + '\n'\
      '         std::integral_constant<int, ' + index + '>,' + '\n'\
      '         std::integral_constant<bool, ' + str(is_last).lower() +\
      '> /*!is_last*/)'
  return_type = create_in_flows_type(output_list)
  return create_method(templates       ='template <typename Out>',
                       signature       =s,
                       return_statement=create_out_flows(output_list),
                       return_type     =return_type)

def create_flow_methods(data, name):
  '''
  Creates methods for inflows and outflows of each stage in the compose using
  the json descriptor.
  '''
  s = ''
  size = data['size']
  for flow_desc,flow_value in data.iteritems():
    if flow_desc not in ['inflow', 'outflow', 'size', 'description']:
      s += create_in_flow_method(flow_desc, flow_value['input'], size) + '\n'
      out_flows = flow_value['output'] if 'output' in flow_value else None
      s += create_out_flow_method(flow_desc, out_flows, size) + '\n'

  return s

def create_top_level_out_port_type_element(flow):
  '''
  An output port can be connected to several flows. This method creates
  the required types for each of those flows.
  '''
  s = 'In'\
      if flow['to'] == 'inflow'\
      else 'typename inner_ports_types<In>::out_port_' + flow['to'] + '_type'
  return create_filter_call_type(flow, s)

def create_top_level_out_port_type(input_list):
  '''
  Create the output port for the struct
  '''
  s = 'using type = '
  if len(input_list) > 1:
    s += 'flows::result_of::concat<\n'
    s += indent(
           ''.join('%s' %',\n'.join(
             map(create_top_level_out_port_type_element, input_list))))
    s += '\n>'
  else:
    s += create_top_level_out_port_type_element(input_list[0])
  return s + ';'

def create_struct(name, body=None, templates=None,
                  private_members=None, public_members=None):
  '''
  Creates a struct with a given name, body, template list, private and public
  members.
  '''
  return \
    (templates + '\n' if templates else '') +        \
    'struct ' + name                        + '\n' + \
    '{'                                     + '\n' + \
    ('private:\n' + indent(private_members) + '\n'   \
       if private_members else '')          +        \
    ('public:\n' + indent(public_members)   + '\n'   \
       if public_members else '')           +        \
    (indent(body) + '\n' if body else '')   +        \
    '};'

def create_inner_ports_types(data, name):
  '''
  Creates the inner_ports_type struct which is used to determine the type
  of each flow in the stage.
  '''
  s = ''
  flows_ids = [a for a in sorted(data) if str(a).isdigit()]
  for flow_id in sorted(flows_ids, key=int):
    if flow_id not in ['size', 'outflow', 'inflow', 'description']:
      s += 'using in_flow_' + flow_id + '_type = ' + \
            create_in_flow_from_out_ports(data[flow_id]['input']) + ';\n'
      s += 'using out_port_' + flow_id + '_type = ' + \
            create_inner_out_port_type(flow_id) + ';\n'

  return create_struct(name='inner_ports_types',
                       templates='template <typename In>',
                       public_members=s)

@doxygen_documentation
def create_flows_struct(data, name):
  '''
  This function creates the flow struct and puts proper documentation using
  the doxygen documentation decorator.

  @param data the input json flow description
  @param name the name of the flow struct to be generated.
  '''
  private_members = \
    'using skeletons_t = typename Compose::skeletons_type;  \n'\
    'Compose const& m_compose;'                          + '\n'

  public_members = \
   'static constexpr std::size_t in_port_size = ' + \
                         str(len(data['inflow']['output'])) + ';\n' + \
   'using in_port_type = ' + create_in_flows_type(data['inflow']['output']) + \
                         ';'
  # add the inner out_port_type struct
  outport_type = \
    create_struct(name='out_port_type',
                  templates='template <typename In>',
                  public_members=
                    create_top_level_out_port_type(data['outflow']['input'])
                 )

  # add the constructor
  const = 'port_types(Compose const& compose)' + '\n'\
          '  : m_compose(compose)'             + '\n'\
          '{ }\n\n'

  # create the body of the inner ports_types class
  body = '\n' + \
         create_inner_ports_types(data, name) + '\n\n' + \
         outport_type                         + '\n\n' + \
         const                                +\
         create_flow_methods(data, name)

  # add the in_port to the ports_types class
  body += create_method(templates       ='',
                        signature       ='in_port(std::size_t lid_offset)',
                        return_statement=
                          create_out_flows(data['inflow']['output']),
                        return_type=
                          'typename port_types<Compose>::in_port_type')

  # add the out_port to the ports_types class
  body += create_method(templates       ='template <typename In>',
                        signature       ='out_port(std::size_t lid_offset)',
                        return_statement=
                          create_in_flows(data['outflow']['input']),
                        return_type     ='typename out_port_type<In>::type',
                        first_line='\n')

  # create the inner ports_types struct
  ports_type = create_struct(name='port_types',
                             templates='template <typename Compose>',
                             private_members=private_members,
                             public_members=public_members,
                             body=body)

  #create the flows struct
  return create_struct(name=name, body=ports_type)



@stapl_skeletons_wrappers
def create_flows(data, name):
  '''
  Your input is supposed to be structured this way:
  the following is a sample of simple piped composition of three elements
  {
    "size"    : YOUR_SEQ_SIZE
    "inflow"  : {"output" : ["to" : "0"]}
    "outflow" : {"input" : ["to" : "2"]}
    "0"       : {"input" : ["to" : inflow"], "output" : ["1"]},
    "1"       : {"input" : ["to" : "0"],     "output" : ["2"]},
    "2"       : {"input" : ["to" : "1"],     "output" : ["outflow"]},
  }

  removing the output field of any of the indexed entries is
  equivalent to bypassing the output of the entry to dontcare_flow
  '''
  s = create_flows_struct(data, name)
  #remove trailing spaces
  return re.sub("\s+$", '\n', s, flags=re.M)

def generate_flows_files(data, flows_name):
  of = open(flows_name + '.hpp', 'w')
  of.write(create_flows(data, flows_name))
  print 'Generated ' + flows_name + '.hpp'
  of.close()

def help():
  print './create_flows.py -i <input-json-file> -o <output-filename>'\
        ' -f <flows-name>'

def main():
  try:
    opts, args = getopt.getopt(
                   sys.argv[1:], 'hf:i:',
                   ['help', 'flows-name', 'flows-json-file'])
  except getopt.error, msg:
    help();
    sys.exit(2)
  input_json_filename = ''
  flows_name = ''
  for o, a in opts:
    if o in ('-h', '--help'):
      help()
      sys.exit(0)
    if o in ('-f', '--flows-name'):
      flows_name = a
    if o in ('-i', '--flows-json-file'):
      input_json_filename = a


  if flows_name == '' or input_json_filename == '':
    help()
    sys.exit(2)

  f = None
  if os.path.isfile(input_json_filename):
    f = open(input_json_filename, 'r')
    data = json.load(f)
    f.close()
    generate_flows_files(data, flows_name)
  else:
    print "The input json file does not exist"
    sys.exit(2)

if __name__ == '__main__':
  main()

