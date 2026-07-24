UCRT64_BIN ?= /ucrt64/bin
CMAKE ?= $(UCRT64_BIN)/cmake.exe
BASH ?= /usr/bin/bash

.PHONY: configure build fg run clean

configure:
	$(CMAKE) --fresh --preset debug

build: configure
	$(CMAKE) --build --preset debug

fg:
	$(BASH) ./scripts/run-flightgear.sh

run:
	$(BASH) ./scripts/run-console.sh

clean:
	rm -rf build/debug
