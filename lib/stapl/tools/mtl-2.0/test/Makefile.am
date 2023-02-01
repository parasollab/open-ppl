AUTOMAKE_OPTIONS = foreign
SUBDIRS = src bin

QUICK	= -qQ

test:
	@- ./bin/make_and_test.pl $(QUICK) -d ./src 

summary:
	@- ./bin/summarize_config.pl -d .. 0
	@- ./bin/summarize_make_and_test.pl .

.PHONY: test summary

CLEANFILES= config_results.txt         make_and_test_results.txt

