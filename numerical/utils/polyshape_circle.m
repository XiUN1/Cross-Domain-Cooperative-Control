function pgon = polyshape_circle(r, x_c, y_c, n)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

t = linspace(0, 2 * pi, n);
x = r * cos(t) + x_c;
y = r * sin(t) + y_c;
pgon = polyshape(x(1:end-1),y(1:end-1));

end