% This file is part of the SPLINTER library.
% Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
%
% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at http://mozilla.org/MPL/2.0/.

% Load SPLINTER (make sure you have SPLINTER in your PATH before this call)
setup()

% Example where the Rosenbrock function is approximated
close all

% Rosenbrock function
rosenbrock = @(x,y) (1-x).^2 + 100*(y-x.^2).^2;

% Coarse grid for with sample points
N = 5;
x = linspace(-2,2,N);
y = linspace(-1,3,N);
[X,Y] = meshgrid(x,y);
Z = rosenbrock(X,Y);

% Fine grid for plotting and evaluation of errors
Nd = 20*N;
xd = linspace(-2,2,Nd);
yd = linspace(-1,3,Nd);
[Xd,Yd] = meshgrid(xd,yd);
Zd = rosenbrock(Xd,Yd);

% Plot function and sample points
figure
surf(Xd,Yd,Zd, 'EdgeColor','none','LineStyle','none')
hold on
stem3(X,Y,Z, 'color', 'black', 'MarkerSize', 6, 'MarkerEdgeColor', 'black', 'MarkerFaceColor','black')
hold off
zlim([0,3000]);
view(210, 30);

% Sample function
xs = zeros(length(x)*length(y), 2);
ys = zeros(length(x)*length(y), 1);
i = 1;
for xi = x
   for yi = y
      xs(i,:) = [xi yi];
      ys(i) = rosenbrock(xi, yi);
      i = i + 1;
   end
end

% Build approximations
bspline1 = BSplineBuilder(xs, ys, 1).build();
bspline2 = BSplineBuilder(xs, ys, 2).build();
bspline3 = BSplineBuilder(xs, ys, 3).build();
bspline4 = BSplineBuilder(xs, ys, 4).build();

% Evaluate approximations and compute errors
approx1 = zeros(Nd,Nd);
error1 = approx1;

approx2 = zeros(Nd,Nd);
error2 = approx2;

approx3 = zeros(Nd,Nd);
error3 = approx3;

approx4 = zeros(Nd,Nd);
error4 = approx4;

i=1;
for xi = xd
    j = 1;
    for yi = yd
        exact = rosenbrock(xi,yi);
        
        approx1(i,j) = bspline1.eval([xi yi]);
        error1(i,j) = (approx1(i,j) - exact);
        approx2(i,j) = bspline2.eval([xi yi]);
        error2(i,j) = (approx2(i,j) - exact);
        approx3(i,j) = bspline3.eval([xi yi]);
        error3(i,j) = (approx3(i,j) - exact);
        approx4(i,j) = bspline4.eval([xi yi]);
        error4(i,j) = (approx4(i,j) - exact);
        
        j = j+1;
    end
   i = i+1;
end

% Plot approximations
figure
subplot(2,2,1);
surf(xd, yd, approx1', 'EdgeColor','none','LineStyle','none')
zlim([0,3000]);
view(210, 30);
title('Linear');

subplot(2,2,2);
surf(xd, yd, approx2', 'EdgeColor','none','LineStyle','none')
zlim([0,3000]);
view(210, 30);
title('Quadratic');

subplot(2,2,3);
surf(xd, yd, approx3', 'EdgeColor','none','LineStyle','none')
zlim([0,3000]);
view(210, 30);
title('Cubic');

subplot(2,2,4);
surf(xd, yd, approx4', 'EdgeColor','none','LineStyle','none')
zlim([0,3000]);
view(210, 30);
title('Quartic');

% Plot errors
figure
subplot(2,2,1);
surf(xd, yd, abs(error1)', 'EdgeColor','none','LineStyle','none')
%zlim([0,3000]);
view(210, 30);
title('Linear');

subplot(2,2,2);
surf(xd, yd, abs(error2)', 'EdgeColor','none','LineStyle','none')
%zlim([0,3000]);
view(210, 30);
title('Quadratic');

subplot(2,2,3);
surf(xd, yd, abs(error3)', 'EdgeColor','none','LineStyle','none')
%zlim([0,3000]);
view(210, 30);
title('Cubic');

subplot(2,2,4);
surf(xd, yd, abs(error4)', 'EdgeColor','none','LineStyle','none')
%zlim([0,3000]);
view(210, 30);
title('Quartic');

% Compute absolute errors
disp('Max error with linear spline:');
abserror1 = max(max(abs(error1)));
abserror1

disp('Max error with quadratic spline:');
abserror2 = max(max(abs(error2)));
abserror2

disp('Max error with cubic spline:');
abserror3 = max(max(abs(error3)));
abserror3

disp('Max error with quartic spline:');
abserror4 = max(max(abs(error4)));
abserror4

% Compute relative errors
rangef = abs(max(max(Zd)) - min(min(Zd)));

disp('Max relative error with linear spline:');
relerror1 = abserror1/rangef;
relerror1

disp('Max relative error with quadratic spline:');
relerror2 = max(max(abs(error2)))/rangef;
relerror2

disp('Max relative error with cubic spline:');
relerror3 = max(max(abs(error3)))/rangef;
relerror3

disp('Max relative error with quartic spline:');
relerror4 = max(max(abs(error4)))/rangef;
relerror4