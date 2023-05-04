# Python header include: Put the path to Python.h here
includes = -I /usr/local/Cellar/python/3.7.4/Frameworks/Python.framework/Versions/3.7/include/python3.7m

# Numpy include: Put the path to numpy/arrayobject.h
includes +=  -I /usr/local/lib/python3.7/site-packages/numpy/core/include

# Python libraries include: Add the path to the directory containing libpython*.a here
includes += -L /usr/local/Cellar/python/3.7.4/Frameworks/Python.framework/Versions/3.7/lib

# Link your Python version
linkings = -lpython3.7

# Compiler definitions
definitions = -std=c++11

# Eigen include
eigen_include = -I /usr/local/include/eigen3

# Executable names for examples (w/o Eigen)
example_execs = minimal modern basic animation nonblock xkcd quiver bar surface subplot fill_inbetween fill update

# Executable names for examples using Eigen
eigen_execs = eigen loglog semilogx semilogy small spy

# Example targets (default if just 'make' is called)
examples: $(example_execs)

# Eigen example targets
eigen: $(eigen_execs)

# All examples
all: examples eigen

# Run all examples
run: run_examples run_eigen

# Compiler instructions for examples
$(example_execs): %: examples/%.cpp matplotlibcpp.h
	g++ $< $(includes) $(linkings) -o examples/$@ $(definitions)

# Run examples
run_examples:
	for exec in $(example_execs); do ./examples/$$exec; done

# Compiler instructions for Eigen examples
$(eigen_execs): %: examples/%.cpp matplotlibcpp.h
	g++ $< $(includes) $(eigen_include) $(linkings) -o examples/$@ $(definitions)

# Run Eigen examples
run_eigen:
	for exec in $(eigen_execs); do ./examples/$$exec; done

# Clean all
clean:
	# -f to silent warnings if file does not exist
	for exec in $(example_execs); do rm -f examples/$$exec; done
	for exec in $(eigen_execs); do rm -f examples/$$exec; done
