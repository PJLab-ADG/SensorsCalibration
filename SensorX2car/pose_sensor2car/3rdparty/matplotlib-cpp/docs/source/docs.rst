.. _docs:

The Docs
********

`matplotlibcpp` namespace
=========================


All functions are organised in the namespace `matplotlibcpp`.
For convenience (and in spirit of the Python norm) we usually
define the abbreviation `plt`:

.. code-block:: cpp

  #include "matplotlibcpp.h"
  namespace plt = matplotlibcpp;

The function can then be accessed via:

.. code-block:: cpp

  matplotlibcpp::plot(x, y);
  plt::loglog(x, y);          // if we defined namespace plt = matplotlibcpp


`Vector` type
=============

.. _STL vector: https://en.cppreference.com/w/cpp/container/vector

.. cpp:type:: Vector

   Functions in the Matplotlib-C++ library are designed to work with
   a generic vector type where possible. All template types named
   `Vector*` must support the following operations.
   See the `STL vector`_ documentation for more detail on the implementation.

   .. note::

      Check the declarations with the STL doc

   .. cpp:type:: double value_type

      Definition of the underlying type, `double` may be replaced with
      another suitable type.

   .. cpp:function:: std::size_t size()

      Return the size of the vector.

   .. cpp:function:: value_type operator[](const std::size_t i)

   .. cpp:function:: value_type at(const std::size_t i)

      Return the `i` th element of the vector.

   .. cpp:function:: value_type* data()

      Return a pointer to the first element of the data in the vector.
      The data must furthermore be stored in a consecutive manner.

   .. cpp:function:: value_type* begin()

      Return a pointer to the first element of the data in the vector.

   .. cpp:function:: value_type* end()

      Return a pointer directly behind the last element of the data in the vector.


Plot commands
=============

.. cpp:namespace:: matplotlibcpp

.. _mpl_plot: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.plot.html

.. cpp:function::
      template <typename VectorX, typename VectorY> \
      bool plot(const VectorX &x, const VectorY &y, const std::string &s = "", \
      const std::map<std::string, std::string> &keywords = {})

   .. image:: ../img/matplotlib_icon.png
      :align: right
      :width: 20px
      :height: 20px
      :target: mpl_plot_

   Plot `y` versus `x`.

   The two vectors :math:`x` and :math:`y` must have the same length.
   The formatting string `s` can specify the colour, markers and style of the
   line.
   The map `keywords` may contain additional named arguments for the plot.

   :tparam VectorX: vector-like type, see :cpp:type:`Vector`
   :tparam VectorY: vector-like type, see :cpp:type:`Vector`
   :param x: :math:`x` data for the plot
   :param y: :math:`y` data for the plot
   :param s: (optional) formatting string, see :ref:`here <style>`
   :param keywords: (optional) map specifying additional keywords, see `here <mpl_plot_>`_
   :returns: true if no error has occured, false otherwise

   **Minimal working example**

   .. code-block:: cpp

      #include <vector>
      #include "matplotlibcpp.h"
      namespace plt = matplotlibcpp;

      int main() {
        std::vector<double> x = {1, 2, 3, 4};
        std::vector<double> y = {1, 4, 9, 16};

        plt::plot(x, y);
        plt::show();

        return 0;
      }

   **Example with formatting strings**

   .. code-block:: cpp

      plt::plot(x, y, "r*");  // Red stars as markers, no line

   .. code-block:: cpp

      plt::plot(x, y, "bo-");  // Blue dots + blue line

   **Example with keywords**

   .. code-block:: cpp

      plt::plot(x, y, "bo-", {{"label", "f(x)"}});  // add the label f(x)
      plt::legend();                                // remember to activate the legend

   .. code-block:: cpp

      plt::plot(x, y, {{"label", "$y = x^2$"}});  // latex is supported
      plt::legend();


.. cpp:function::
      template <typename VectorY> \
      bool plot(const VectorY &y, const std::string &format = "", \
                const std::map<std::string, std::string> &keywords = {})

   .. image:: ../img/matplotlib_icon.png
      :align: right
      :width: 20px
      :height: 20px
      :target: mpl_plot_

   Plot `y`.

   For a vector :math:`y` of size :math:`n`, the :math:`x` data
   is set to :math:`{0, ..., n - 1}`.
   The formatting string `s` can specify the colour, markers and style of the
   line.
   The map `keywords` may contain additional named arguments for the plot.

   :tparam VectorY: vector-like type, see :cpp:type:`Vector`
   :param y: :math:`y` data for the plot
   :param s: (optional) formatting string, see :ref:`here <style>`
   :param keywords: (optional) map specifying additional keywords, see `here <mpl_plot_>`_
   :returns: true if no error has occured, false otherwise

   **Examples**

   .. code-block:: cpp

      #include <vector>
      #include "matplotlibcpp.h"
      namespace plt = matplotlibcpp;

      int main() {

        std::vector<int> y = {1, 2, 3};
        plt::plot(y, "bo-");
        plt::show();

        return 0;
      }

   .. code-block:: cpp

      Eigen::VectorXd y = {1, 2, 3};
      plt::plot(y, {{"label", "1 to 3"}});
      plt::show();

.. _mpl_loglog: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.loglog.html

.. cpp:function::
      template <typename VectorX, typename VectorY> \
      bool loglog(const VectorX &x, const VectorY &y, const std::string &s = "", \
                  const std::map<std::string, std::string> &keywords = {})

   .. image:: ../img/matplotlib_icon.png
      :align: right
      :width: 20px
      :height: 20px
      :target: mpl_loglog_

   Plot `y` versus `x` in double logarithmic scale.

   See :cpp:func:`plot` for explanation of the parameters.

   .. note::
      All following plots will be in double logarithmic scale,
      also calls to `plot`.

   **Example**

   .. code-block:: cpp

      #include <Eigen/Dense>
      #include "matplotlibcpp.h"
      namespace plt = matplotlibcpp;

      int main() {
        int n = 5000;
        Eigen::VectorXd x(n), y(n), z(n), w = Eigen::VectorXd::Ones(n);
        for (int i = 0; i < n; ++i) {
          double value = (1.0 + i) / n;
          x(i) = value;
          y(i) = value * value;
          z(i) = value * value * value;
        }

        plt::loglog(x, y);         // f(x) = x^2
        plt::loglog(x, w, "r--");  // f(x) = 1, red dashed line
        plt::loglog(x, z, "g:", {{"label", "$x^3$"}}); // f(x) = x^3, green dots + label

        plt::title("Some functions of $x$"); // add a title
        plt::show();
      }

.. cpp:function::
      template <typename VectorY> \
      bool loglog(const VectorY &y, const std::string &s = "", \
                  const std::map<std::string, std::string> &keywords = {})

   .. image:: ../img/matplotlib_icon.png
      :align: right
      :width: 20px
      :height: 20px
      :target: mpl_loglog_

   Plot `y` in double logarithmic scale.

   See :cpp:func:`plot` for explanation of the parameters.

   .. note::
      All following plots will be in double logarithmic scale,
      also calls to `plot`.

   **Examples**

   Assuming ``vector`` and ``matplotlibcpp`` import and the namespace
   definition ``plt = matplotlibcpp``.

   .. code-block:: cpp

      std::vector<int> y = {1, 10, 100, 1000};
      plt::loglog(y);

   .. code-block:: cpp

      std::vector<double> y1 = {1, 2, 4},
                          y2 = {1, 3, 9};
      plt::loglog(y, "bo-", {{"label", "powers of 2"}});
      plt::plot(y, "ro-", {{"label", "powers of 3"}});  // also in loglog scale


.. _mpl_semilogx: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.semilogx.html

.. cpp:function::
      template <typename VectorX, typename VectorY> \
      bool semilogx(const VectorX &x, const VectorY &y, const std::string &s = "", \
                    const std::map<std::string, std::string> &keywords = {})

   .. image:: ../img/matplotlib_icon.png
      :align: right
      :width: 20px
      :height: 20px
      :target: mpl_semilogx_

   Plot `y` versus `x` in logarithmic `x` and linear `y` scale.

   See :cpp:func:`plot` for explanation of the parameters.

   .. note::
      All following plots will inherit the logarithmic `x` scale,
      also calls to `plot`.

.. cpp:function::
      template <typename VectorY> \
      bool semilogx(const VectorY &y, const std::string &s = "", \
                    const std::map<std::string, std::string> &keywords = {})

   .. image:: ../img/matplotlib_icon.png
      :align: right
      :width: 20px
      :height: 20px
      :target: mpl_semilogx_

   Plot `y` in logarithmic `x` and linear `y` scale.

   See :cpp:func:`plot` for explanation of the parameters.

   .. note::
      All following plots will inherit the logarithmic `x` scale,
      also calls to `plot`.


.. _mpl_semilogy: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.semilogy.html

.. cpp:function::
      template <typename VectorX, typename VectorY> \
      bool semilogy(const VectorX &x, const VectorY &y, const std::string &s = "", \
                    const std::map<std::string, std::string> &keywords = {})

   .. image:: ../img/matplotlib_icon.png
      :align: right
      :width: 20px
      :height: 20px
      :target: mpl_semilogy_

   Plot `y` versus `x` in linear `x`  and logarithmic `y` scale.

   See :cpp:func:`plot` for explanation of the parameters.

   .. note::
      All following plots will inherit the logarithmic `y` scale,
      also calls to `plot`.

.. cpp:function::
      template <typename VectorY> \
      bool semilogy(const VectorY &y, const std::string &s = "", \
                    const std::map<std::string, std::string> &keywords = {})

   .. image:: ../img/matplotlib_icon.png
      :align: right
      :width: 20px
      :height: 20px
      :target: mpl_semilogy_

   Plot `y` in linear `x` and logarithmic `y` scale.

   See :cpp:func:`plot` for explanation of the parameters.

   .. note::
      All following plots will inherit the logarithmic `y` scale,
      also calls to `plot`.

.. _mpl_text: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.text.html

.. cpp:function::
      template <typename Numeric> \
      void text(Numeric x, Numeric y, const std::string &s = "")

   .. image:: ../img/matplotlib_icon.png
      :align: right
      :width: 20px
      :height: 20px
      :target: mpl_text_

   Place text at location :math:`(x,y)`.

   :tparam Numeric: A scalar-like type
   :param x: The :math:`x` location of the text
   :param y: The :math:`y` location of the text
   :param s: The text to be placed in the plot

   **Example**

   .. code-block:: cpp

      #include <vector>
      #include "matplotlibcpp.h"
      namespace plt = matplotlibcpp;

      int main() {

        std::vector<double> x = {0.1, 0.2, 0.5};
        plt::plot(x, "s");
        plt::text(1.0, 0.1, "Text under a square");
        plt::show();

        return 0;
      }


.. _layout:

Figure commands
===============

.. _mpl_figure: https://matplotlib.org/3.1.0/api/_as_gen/matplotlib.pyplot.figure.html

.. cpp:function::
      inline long figure(long number = -1)

    .. image:: ../img/matplotlib_icon.png
       :align: right
       :width: 20px
       :height: 20px
       :target: mpl_figure_

    Initialise a new figure with the ID `number`.

    :param number: The number of the figure. If set to `-1` default numbering
                   (increasing from `0` on) is used
    :return: The number of the figure

.. _mpl_fignum_exists: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.fignum_exists.html

.. cpp:function::
      inline bool fignum_exists(long number)

    .. image:: ../img/matplotlib_icon.png
       :align: right
       :width: 20px
       :height: 20px
       :target: mpl_fignum_exists_

    Check if a figure of given number exists.

    :param number: The number of the figure
    :return: true, if a figure with given number exists, false otherwise

.. cpp:function::
      inline void figure_size(size_t w, size_t h)

    Call `plt::figure()` and set the figure size to `w` x `h` pixels.

    :param w: The width of the figure in pixels
    :param h: The height of the figure in pixels

.. _mpl_legend: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.legend.html

.. cpp:function::
      template <typename Vector = std::vector<double>> \
      inline void legend(const std::string &loc = "best", \
                         const Vector &bbox_to_anchor = Vector())

    .. image:: ../img/matplotlib_icon.png
       :align: right
       :width: 20px
       :height: 20px
       :target: mpl_legend_

    Enable the figure legend.

    :tparam Vector: vector-like type, see :cpp:type:`Vector`, defaults
                to `std::vector<double>`
    :param loc: The location of the legend. May be any of:
                "best", "upper left", "upper center", "upper left",
                "center left", "center", "center right" (= "right"),
                "lower left", "lower center", "lower right"
    :param bbox_to_anchor:
               If set to a vector of length 2 or 4 it
               specifies the location (and size) of the legend's bounding box.
               Format is (`x`, `y`) or (`x`, `y`, `width`, `height`).
               The coordinates are interpreted in the same units as the
               plot axes (thus no normalised coordinates)

    **Example**

    .. code-block:: cpp

      // Put the legend in the center of the bottom right quadrant.
      // First argument: loc, second: bbox_to_anchor
      plt::legend("center", {0.5, 0, 0.5, 0.5});


.. _mpl_xlim: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.xlim.html

.. cpp:function::
      template <typename Numeric> \
      void xlim(Numeric left, Numeric right)

    .. image:: ../img/matplotlib_icon.png
       :align: right
       :width: 20px
       :height: 20px
       :target: mpl_xlim_

    Set the `x` axis limits.

    :tparam Numeric: A scalar-like type
    :param left: The left axis limit
    :param right: The right axis limit

.. _mpl_ylim: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.ylim.html

.. cpp:function::
      template <typename Numeric> \
      void ylim(Numeric bottom, Numeric top)

    .. image:: ../img/matplotlib_icon.png
       :align: right
       :width: 20px
       :height: 20px
       :target: mpl_ylim_

    Set the `y` axis limits.

    :tparam Numeric: A scalar-like type
    :param bottom: The bottom axis limit
    :param top: The top axis limit

.. cpp:function::
      inline double *xlim()

    Get the `x` axis limits.

    :return: A pointer to an array of length 2 containing `[left, right]`


.. cpp:function::
      inline double *ylim()

    Get the `y` axis limits.

    :return: A pointer to an array of length 2 containing `[bottom, top]`


.. _mpl_title: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.title.html

.. cpp:function::
      inline void title(const std::string &titlestr, \
                        const std::map<std::string, std::string> &keywords = {})

   .. image:: ../img/matplotlib_icon.png
      :align: right
      :width: 20px
      :height: 20px
      :target: mpl_title_

   Set the title of the plot.

   :param titlestr: Title of the plot
   :param keywords: Additional keywords, see `here <mpl_title_>`_ for a list


.. _mpl_suptitle: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.suptitle.html

.. cpp:function::
        inline void suptitle(const std::string &suptitlestr, \
                             const std::map<std::string, std::string> &keywords = {})

   .. image:: ../img/matplotlib_icon.png
      :align: right
      :width: 20px
      :height: 20px
      :target: mpl_suptitle_

   Add a centered title to the figure.

   :param suptitlestr: Title of the figure
   :param keywords: Additional keywords, see `here <mpl_suptitle_>`_ for a list

.. _mpl_axis: https://matplotlib.org/api/_as_gen/matplotlib.pyplot.axis.html

.. cpp:function::
      inline void axis(const std::string &option)

   .. image:: ../img/matplotlib_icon.png
      :align: right
      :width: 20px
      :height: 20px
      :target: mpl_suptitle_

   Set some axis properties.

   :param option: The option to activate

   ========= ================================================
   option     Result
   ========= ================================================
   `on`      Turn on axis lines and labels
   `off`     Turn off axis lines and labels
   `equal`	  Set equal scaling (i.e., make circles circular) by changing axis limits.
   `scaled`	Set equal scaling (i.e., make circles circular) by changing dimensions of the plot box.
   `tight` 	Set limits just large enough to show all data.
   `auto`	  Automatic scaling (fill plot box with data).
   `image`	  `scaled` with axis limits equal to data limits.
   `square`	Square plot; similar to `scaled`, but initially forcing same x- and y-axis length.
   ========= ================================================

.. _mpl_savefig: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html

.. cpp:function::
      inline void savefig(const std::string &filename, \
                          const std::map<std::string, std::string> &keywords = {})

   .. image:: ../img/matplotlib_icon.png
      :align: right
      :width: 20px
      :height: 20px
      :target: mpl_savefig_

   Save the current figure.

   Supported file types depend on the user backend, but usually
   contain `pdf`, `eps` and `png`. To find all supported formats try

   .. code-block:: bash

     $ python3
     >>> import matplotlib.pyplot as plt
     >>> plt.gcf().canvas.get_supported_filetypes_grouped()

   :param filename: Save the figure to `filename` (must contain file format)
   :param keywords: Additional keywords, see `Other Parameters` `here <mpl_savefig>`_ for a complete list

   **Examples**

   .. code-block:: cpp

    plt::plot(x, y);
    plt::savefig("plot.pdf");

   Always the current state of the figure is stored.

   .. code-block:: cpp

     plt::plot(time, apple_sales);
     plt::savefig("sales.pdf");  // contains only apple_sales
     plt::plot(time, kiwi_sales);
     plt::savefig("sales.pdf");  // contains apple and kiwi sales

   Calling `plt::show()` clears the plot!

   .. code-block:: cpp

     plt::plot(x, y);
     plt::show();
     plt::savefig("is_this_empty.pdf");  // yes, this will be empty

     plt::plot(x, y);
     plt::savefig("this_isnt_empty.pdf");  // always call savefig *before* show
     plt::show();

   Optimally use the available canvas space with `{{"bbox_inches", "tight"}}`.
   This can be useful if e.g. the axis labels are too far outside and get cut off.

   .. code-block:: cpp

     plt::savefig("fig.pdf", {{"bbox_inches", "tight"}});


.. _mpl_show: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.show.html

.. cpp:function::
      inline void show(const bool block = true)

   .. image:: ../img/matplotlib_icon.png
      :align: right
      :width: 20px
      :height: 20px
      :target: mpl_show_

   Display the figure.

   :param block: If true, the execution of the code is stopped until the
                 displayed figure is closed. Otherwise the code is not stopped.
                 Depending on the backend, figures might not get displayed
                 at all.
