import os
from xml.etree import ElementTree as ET

XML = ET.parse('TEMPLATE_experiment-setup.xml')

# Array of random seeds
seeds = ['123456789']

# Array of strategies
strategies = ['DynamicRegionRRT']

# Array of queries; start-query, then goal query
strategies = ['max-ClearanceMethod', 'DynamicRegionRRT']
queries = ['-36 -18 0 0 0 0', '-8 14 0 0 0 0']

# Variables for elements
mp = XML.getroot()
listMp = mp.getchildren()
listProbl = listMp[0]
listTask = listProbl[2]
listStartConstraints = listTask[0]
listGoalConstraints = listTask[1]
listLib = listMp[1]
tagSolver = listLib[12]
listSampler = listLib[3]
tagDRRRT = listSampler.find('DynamicRegionSampler')

# Iterates through the strategies, seeds, and queries to create
# new xml files for each
for i in range(len(strategies)):
  for j in range(len(seeds)):
    for k in range((len(queries)/2)):

      # loop variables
      filename = strategies[i]+'-'+seeds[j]+'-'+str(k)
      trFilename = 'TR-'+strategies[i]+'-'+str(k)
      q_index = 2*k

      tagSolver.attrib['seed'] = seeds[j]

      if(strategies[i] == 'max-ClearanceMethod'):
        tagSolver.attrib['mpStrategyLabel'] = 'DynamicRegionRRT'
        tagDRRRT.attrib['biasMetric'] = 'clearance'
        tagDRRRT.attrib['clearanceValue'] = 'max'
      if(strategies[i] == 'min-ClearanceMethod'):
        tagSolver.attrib['mpStrategyLabel'] = 'DynamicRegionRRT'
        tagDRRRT.attrib['biasMetric'] = 'clearance'
        tagDRRRT.attrib['clearanceValue'] = 'min'
      if(strategies[i] == 'DynamicRegionRRT' ):
        tagDRRRT.attrib['biasMetric'] = 'default'
        tagSolver.attrib['mpStrategyLabel'] = 'DynamicRegionRRT'
      if(strategies[i] == 'RRT'):
        tagSolver.attrib['mpStrategyLabel'] = 'RRT'

      tagSolver.attrib['baseFilename'] = trFilename
      listStartConstraints[0].attrib['point'] = queries[q_index]
      listGoalConstraints[0].attrib['point'] = queries[q_index+1]

      # Write all the changes to XML to a new file with template name
      XML.write('ENV-Setup-'+filename+'.xml')
      os.system('~/Research/PMPL/pmpl/src/pmpl -f ' + 'ENV-Setup-'+filename+'.xml >> ' + 'ENV-Setup-'+filename+'.log')


