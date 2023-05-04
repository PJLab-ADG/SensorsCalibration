.. _compiling:

Compiling a program
*******************

Requirements
============

Matplotlib for C++ requires a working Python installation as well as
Matplotlib. Python2.7 and Python3 (>= 3.6) have been tested, but
other versions should work as well. In the linking process the exact
version of Python to use can be specified by linking the according library.

On Unix it is recommended to install Python via the package manager to
assert that all dependencies are installed properly.

.. code-block:: bash

   <package-manager> install python3 python3-dev  # or -devel depending on the platform

If Python is installed from source problems in the linking may occur.
How to resolve these is explained in the next section, or in
:ref:`this <pyfromsource>` code-block.

Install matplotlib via pip

.. code-block:: bash

  pip3 install matplotlib  # or pip for Python 2

Includes and Linking
====================


The header ``matplotlibcpp.h`` depends on the Python header, ``Python.h``,
the corresponding Python library ``libpython``, and on ``numpy/arrayobject.h``.
If not in the standard include paths, the paths to the header files,
the path to the library, and the library itself have to be specified
for the compiler using the options ``-I``, ``-L`` and ``-l`` respectively.
Note, that all Python constituents should be of the same Python version.
Matplotlib for C++ supports both, Python 2.7 and Python 3 versions.

In detail:

  - The Python header ``Python.h``

    The Python header comes with the Python installation. If it cannot be
    found on your system try installing the Python development packages.
    The location of this header has to be specified using the option ``-I``.

    Typical locations:

    - Linux: `/usr/local/include/python3.7`
    - Mac: if installed with Homebrew `/usr/local/Cellar/python/3.7.3/Frameworks/Python.framework/Versions/3.7/include/python3.7m`

  - The Python library ``libpython*.so``

    The program has to be linked against the compiled Python library.
    Depending on the Python version the name of the library differs, for
    Python 3.7 it is ``libpython3.7.so`` (or ``libpython3.7m.so``).
    Then link the library by specifying ``-lpython3.7`` (or ``-lpython3.7m``).

    Additionally to the linking the location of the library must be specified
    if not installed in the usual directory. For Linux systems this is
    usually not necessary, for Mac however it mostly is.
    The location of the library has to be specified using the option ``-L``.

    If Python has not been installed using the package manager (but e.g.
    from source) twofold problems with linking the library can occur.
    The first are missing dependencies of the Python library, these can be
    added via ``-lpthread -lutil -ldl``.
    The second is that dynamic libraries have to be exported which is
    resolved by adding ``-Xlinker -export-dynamic``.

    Typical locations:

    - Linux: Path usually already included
    - Mac: `/usr/local/Cellar/python/3.7.3/Frameworks/Python.framework/Versions/3.7/lib`

  - Numpy array ``numpy/arrayobject.h``

    By default Matplotlib for C++ uses Numpy arrays. This requires the above
    header file. However it is possible to avoid this header by defining
    ``-DWITHOUT_NUMPY``.

    - Linux: `/usr/local/lib/python3.7/site-packages/numpy/core/include`
    - Mac: If installed via Homebrew, same as for Linux.

**Examples**

On Linux with the GNU compiler ``g++`` and
C++11.

.. code-block:: bash

   # using Python 2.7
   g++ main.cpp -std=c++11 -I/usr/local/include/python2.7 \
     -I/usr/local/lib/python2.7/site-packages/numpy/core/include -lpython2.7

.. code-block:: bash

   # using Python3.7 and no Numpy
   g++ main.cpp -std=c++11 -DWITHOUT_NUMPY -I/usr/local/include/python2.7 -lpython2.7

On Mac with the GNU compiler ``g++`` and C++14.

.. code-block:: bash

   g++ main.cpp -std=c++14 \
    -I /usr/local/Cellar/python/3.7.3/Frameworks/Python.framework/Versions/3.7/include/python3.7m \
    -I /usr/local/lib/python3.7/site-packages/numpy/core/include \
    -L /usr/local/Cellar/python/3.7.3/Frameworks/Python.framework/Versions/3.7/lib \
    -lpython3.7

With exporting dynamic libraries and linking to all dependencies of
the Python library on a Linux system:

.. _pyfromsource:

.. code-block:: bash

   g++ main.cpp -std=c++11 -I/usr/local/include/python3.7m \
     -I/usr/local/lib/python3.7/site-packages/numpy/core/include \
     -lpython3.7m \
     -lpthread -lutil -ldl \ # library dependencies
     -Xlinker -export-dynamic \ # export dynamic libraries
