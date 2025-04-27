DIFF = diff
RM = rm -f
CXXFLAGS += -ggdb -Wall -I.

default: regressiontest

.PHONY : MicroNMEA.h
MicroNMEA.h : ../../src/MicroNMEA.h
	echo '' > $@
	echo '#include "arduino_compat.h"' >> $@
	echo '#include <iostream>' >> $@
	echo 'using namespace ::std;' >> $@
	cat $< >> $@

MicroNMEA.cpp : ../../src/MicroNMEA.cpp
	cp $< $@

MicroNMEA.o : MicroNMEA.cpp MicroNMEA.h
	$(CXX) -DMICRONMEA_DEBUG $(CXXFLAGS) -c $< -o $@

test_MicroNMEA.o : test_MicroNMEA.cpp MicroNMEA.h

test_MicroNMEA : test_MicroNMEA.o MicroNMEA.o
	$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

.PHONY : run
run : test_MicroNMEA 
	./test_MicroNMEA < nmea_sentences.txt
# Or run with electric fence to catch segmentation faults
# 	LD_PRELOAD=libefence.so ./test_MicroNMEA < nmea_sentences.txt

# Regression testing. Run as "make regressiontest", should display
# "TEST PASSED" if everything ok.
.PHONY : regressiontest
regressiontest :
	$(MAKE) realclean
	$(MAKE) test_MicroNMEA
	./test_MicroNMEA < nmea_sentences.txt > test_MicroNMEA.regressiontest.tmp
	echo TEST RUN
	$(DIFF) -s -u test_MicroNMEA.regressiontest test_MicroNMEA.regressiontest.tmp
	-$(RM) test_MicroNMEA.regressiontest.tmp
	@echo
	@echo TEST PASSED

.PHONY : clean
clean : 
	-$(RM) *.o MicroNMEA.h MicroNMEA.cpp test_MicroNMEA.regressiontest.tmp

.PHONY : realclean
realclean : clean
	-$(RM) -f test_MicroNMEA 
