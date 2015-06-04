ifndef Script
  Script = tests.sh
  #Script = parallelTests.sh
endif

ifndef Test
  Test = CfgTests
  #Test = ParallelCfgTests
  #Test = ReachableVolumeTests
endif

test:
	./${Script} ${Test}

clean:
	rm -f Scratch/*
