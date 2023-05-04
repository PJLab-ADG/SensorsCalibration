##C interface
As part of making the MATLAB and Python interface we have also made a C interface to the library. The MATLAB and Python interfaces make all their calls through this interface, but you can still use it if you want to. Note that almost all functions emulate object oriented languages by taking a splinter_obj_ptr as the first argument, and then the rest of the arguments after that.
splinter_obj_ptr is currently defined as
```c
typedef void *splinter_obj_ptr;
```

```c
#include <stdio.h>
#include <stdlib.h>
#include <cinterface/cinterface.h>

double f(double x, double y)
{
        return x*x*y + y*y*y;
}

int main(int argc, char **argv)
{
        splinter_obj_ptr datatable = splinter_datatable_init();
        printf("%s\n", splinter_get_error_string());

        int x_grid = 10;
        int y_grid = 10;
        int n_samples = x_grid * y_grid;

        double *samples = (double *) malloc(sizeof(double) * n_samples * 3);
        int sampleIdx = -1;
        for (int x = 0; x < x_grid; ++x)
        {
                for (int y = 0; y < y_grid; ++y)
                {
                        samples[++sampleIdx] = x;
                        samples[++sampleIdx] = y;
                        samples[++sampleIdx] = f(x, y);
                }
        }

        splinter_datatable_add_samples_row_major(datatable, samples, n_samples, 2);
        if (splinter_get_error()) {
                printf("%s\n", splinter_get_error_string());
        }

        splinter_obj_ptr bspline_builder = splinter_bspline_builder_init(datatable);
        if (splinter_get_error()) {
                printf("%s\n", splinter_get_error_string());
        }
        
        unsigned int degrees[2] = {3, 3};
        splinter_bspline_builder_set_degree(bspline_builder, degrees, 2);
        if (splinter_get_error()) {
                printf("%s\n", splinter_get_error_string());
        }

        splinter_obj_ptr bspline = splinter_bspline_builder_build(bspline_builder);
        if (splinter_get_error()) {
                printf("%s\n", splinter_get_error_string());
        }

        double x_eval[] = {0.1, 0.5};
        double *val = splinter_bspline_eval_row_major(bspline, x_eval, 2);
        if (splinter_get_error()) {
                printf("%s\n", splinter_get_error_string());
        }

        printf("Approximated value at (%f, %f): %f\n", x_eval[0], x_eval[1], val[0]);
        printf("Exact value at (%f, %f): %f\n", x_eval[0], x_eval[1], f(x_eval[0], x_eval[1]));

        return 0;
}

/* Output:
No error.
No error.
No error.
No error.
Approximated value at (0.100000, 0.500000): 0.130000
Exact value at (0.100000, 0.500000): 0.130000
*/
```
Example compiled with: c99 test.c -I../../include/ -L. -lsplinter-3-0 && ./a.out
from ~/SPLINTER/build/release, where libsplinter-3-0.so was located.

If you run into problems with running the produced executable, you should have a look [here](http://tldp.org/HOWTO/Program-Library-HOWTO/shared-libraries.html).
