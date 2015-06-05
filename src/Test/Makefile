ifndef parallel
  parallel = 0
endif

ifndef Test
  Test = CfgTests
  #Test = ParallelCfgTests
  #Test = ReachableVolumeTests
endif

test:
	./test.sh ${Test} ${parallel}

clean:
	rm -f Scratch/*
