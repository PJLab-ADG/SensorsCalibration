## Changelog

#### Version 3.0
- Removed class for polynomial regression (PolynomialRegression).
- Removed class for radial basis function approximation (RBFApproximant)
- Updated MATLAB and Python examples
- Added a new builder class for B-splines (BSpline::Builder)
- Changed default architecture to x86-64
- Added support for equidistant knot vectors (still experimental)
- Added support for selecting the number of basis functions when constructing a B-spline
- Added support for building splines with a regularization term (as an alternative to the P-spline)
- Replaced all assertions with exceptions
- Reworked MATLAB, Python, and C interfaces
- Changed to Eigen version 3.2.8
- Updated documentation

#### Version 2.0
- Automatic knot vector selection using moving average
- Added PolynomialRegression (including MatLab interface)
- Added saving and loading of PSpline, RadialBasisFunction and PolynomialRegression
- Reworked serialization (~5% faster serialization now)
- Added loading of serialized objects in MatLab interface: b = BSpline('saved_bspline.bspline')
- Added loading and saving of DataTable to MatLab interface
- Refactored MatLab interface backend (no end user visible changes)
- Integrated the Catch testing framework and added extensive testing of approximation, serialization and datatable set operation features
- Added set operations (union and complement) to DataTable (as operator+ and operator-)
- Added script (scripts/build_release.sh) for easier compilation (especially on Windows with MSVC)
- Improved compilation documentation
- Renamed API macro to SPLINTER_API to avoid name collisions with users namespace
- Fixed BSpline hessian not being symmetric
- Added evalJacobian implementations for PolynomialRegression and RadialBasisFunctions
- Added standard C++11 types to Approximant interface
- Refactored B-spline code.
- Added Python interface to the library
- Added batch evaluation to MatLab interface
- Renamed Approximant subclasses class names, example: BSpline -> BSplineApproximant

#### Version 1.3
- Library renamed SPLINTER (from Splinter)
- Namespace changed from Splinter to SPLINTER to reflect the name change
- BSplines can now be decomposed into Bezier form (decomposeBezierForm())
- Spline class renamed Approximant to open for other forms of interpolation in the future
- Added getNumVariables to the Approximant interface
- rbspline.* renamed to radialbasisfunction.*
- Removed QUADRATIC_FREE and CUBIC_FREE BSpline types, available types are now LINEAR, QUADRATIC, CUBIC and QUARTIC
- Added MatLab interface
- Improved build system (CMake code)

#### Version 1.2
- Renamed library from multivariate-splines to Splinter.
- Added saving and loading of BSplines and DataTables in binary form.

#### Version 1.1
- Quadratic B-splines are now exposed to the user
- Structured exceptions implemented all over the library
- Internal logging only in debug mode
- Automatically detect Eigen when compiling (thanks skific)
- Replaced stringstream stod and stoi implementation
- Minor bug-fixes
- Clean-up: mostly formatting and renaming

#### Version 1.0
- Initial release
