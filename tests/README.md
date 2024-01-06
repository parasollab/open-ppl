# Tests

## Running Strategy Tests

This will run the Basic PRM Tests by default
```
./ppl_strategy_tests
```

or by running all files (lines) defined in Tests text files such as **CfgTests**. Other files are for example **LazyTests**, **TogglePRMTest**.
```
./ppl_strategy_tests -F CfgTests
```
Note that currently only -F flag is defined.

## Running Unit Tests

This will run the default **FullUnitTest.xml** XML file already available in `pmpl/tests` folder
```
./ppl_unit_tests
```

or by running XML file directly by providing the path (here Linux path) to the file
```
./ppl_unit_tests -U /home/<user>/pmpl/tests/FullUnitTest.xml
```
Note that currently only -U flag is defined for unit tests.

## Testing framework

Testing framework uses [Catch2](https://github.com/catchorg/Catch2) and PMPL native test interface. **Strategy test** part is defined in `tests_strategy_main.cpp` and `StrategyTests/StrategyTests.cpp` files uses a single TEST_CASE. **Unit test** part on the other hand uses four [multi file](https://github.com/catchorg/Catch2/blob/devel/examples/020-TestCase-1.cpp) TEST_CASES:
- Geometry TESTS
- Behaviors TESTS
- MPLibrary TESTS
- TMPLibrary TESTS

Note. TMPLibrary TESTS are work-in-progress marked with a [special tag](https://github.com/catchorg/Catch2/blob/devel/docs/test-cases-and-sections.md) `[!mayfail]` which means that tests don't fail if any given assertion fails.

## Registering tests

### MixSamplerTest Example using Traits

1. add MixSamplerTest header file with MixSamplerTest class, helper function and test interface to the appropriate folder, here `tests/MPLibrary/Samplers/`
2. include sampler header file in **TestTraits** (`tests/Traits/`)
3. add MixSamlerTest in list of samplers types available in PMPL world.
