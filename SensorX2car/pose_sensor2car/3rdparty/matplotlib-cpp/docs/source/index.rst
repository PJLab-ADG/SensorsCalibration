.. Matplotlib for C++ documentation master file, created by
   sphinx-quickstart on Thu Jul 18 20:49:11 2019.

Home
****

This is the documentation to *Matplotlib for C++*, a C++ wrapper
for Python's `matplotlib` (MPL) plotting library.

The code is organised in `this <https://github.com/Cryoris/matplotlib-cpp>`_
GitHub repository, which is a fork of
`that <https://github.com/lava/matplotlib-cpp>`_ repository.

.. note::

  *This is*:
  A lightweight, easy-to-use interface to create stylish and clean plots in C++
  using basic MPL commands.

  *This is* **not**:
  A translation of MPL to C++.

How to use this documentation
=============================

.. _mpl_doc: https://matplotlib.org/3.1.1/index.html

Function definitions
++++++++++++++++++++

This is the core of the documentation, located at :ref:`docs`.
To find the definition and explanations for a special command use
the search field on the top left, since this page can get a bit lengthy.

Bear in mind, that `matplotlibcpp` is a C++ wrapper to the Python
library MPL. Thus, to learn more about the functions that
are eventually called the `matplotlib documentation <mpl_doc_>`_ might be useful.
Most functions have a link to the MPL function they call, marked
with the MPL logo:

.. image:: ../img/matplotlib_icon.png
  :align: center
  :width: 20px
  :height: 20px
  :alt: Clicking here leads to the corresponding function definition in MPL
  :target: mpl_doc_

However, the function signatures might differ and Matplotlib for C++ does
*not* support the full functionality of MPL.
The purpose is providing an easy-to-use wrapper to MPL in C++, not
to fully translate the library.

Help for compiling
++++++++++++++++++

The section :ref:`compiling` explains the compilations of
a program using the ``matplotlibcpp.h`` header.

Style of a line
+++++++++++++++

Refer :ref:`style` to tune the appearance of the lines you plot.

Frequent problems
+++++++++++++++++

:ref:`problems` lists typical problems with Matplotlib for C++
and (hopefully) how to resolve them.

.. tip::

  Criticism (preferably constructive), ideas and contributions
  are welcome! For contact, see :ref:`questions`.

.. toctree::
   :maxdepth: 2
   :caption: Content

   examples
   compiling
   docs
   style
   problems
   license
   todo
