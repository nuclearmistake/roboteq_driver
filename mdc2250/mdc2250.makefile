all: mdc2250

install:
	cd build && make install

uninstall:
	cd build && make uninstall

mdc2250:
	@mkdir -p build
	-mkdir -p bin
	cd build && cmake $(CMAKE_FLAGS) ..
ifneq ($(MAKE),)
	cd build && $(MAKE)
else
	cd build && make
endif

clean:
	-cd build && make clean
	rm -rf build bin lib

.PHONY: doc
doc:
	@doxygen doc/Doxyfile
ifeq ($(UNAME),Darwin)
	@open doc/html/index.html
endif

.PHONY: test
test:
	@mkdir -p build
	@mkdir -p bin
	cd build && cmake $(CMAKE_FLAGS) -DMDC2250_BUILD_TESTS=1 -DMDC2250_BUILD_EXAMPLES=1 ..
ifneq ($(MAKE),)
	cd build && $(MAKE)
else
	cd build && make
endif
	cd bin && ./mdc2250_listener_tests