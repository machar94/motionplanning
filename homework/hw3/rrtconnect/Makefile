
all:
	@mkdir -p build;
	cd build && cmake .. && $(MAKE) $(PARALLEL_JOBS)

install:
	cd build && $(MAKE) $(PARALLEL_JOBS) install

uninstall:
	cd build && $(MAKE) uninstall

clean:
	-cd build && $(MAKE) clean

