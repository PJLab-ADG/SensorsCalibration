.. _problems:

Frequent problems
*****************

Figure layout
=============

Cutoff labels
+++++++++++++

If the axis ticks are too long, the axis labels might be outside
of the figure. When saving the figure, the labels can be cutoff.

For instance the following code produces a cutoff y-axis label.

.. code-block:: cpp


  #include "../../matplotlibcpp.h"
  #include <Eigen/Dense>
  #include <iostream>

  namespace plt = matplotlibcpp;

  int main() {
    Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(200, 0, 6);
    Eigen::VectorXd y, z;

    // y = exp(sin(x)), z = exp(cos(z))
    y = x.array().sin().exp().matrix();
    z = x.array().cos().exp().matrix();

    plt::figure();

    plt::loglog(x, y);
    plt::loglog(x, z);

    plt::xlabel("Time in lecture");
    plt::ylabel("Student confusion");

    plt::grid();
    plt::savefig("loglog.pdf"); // !
  }

The output is

.. image:: ../img/problems/layout/cutoff_labels.pdf

To fix this problem we can tell MPL to use the available space
on the empty sides of the figure, which can be done
either by a call to ``tight_layout`` or specifiying ``bbox_inches="tight"``
in ``savefig``.

Thus the issue is fixed by

.. code-block:: cpp

  plt::savefig("loglog.pdf", {{"bbox_inches", "tight"}}); // !

.. image:: ../img/problems/layout/cutoff_labels_fixed.pdf
