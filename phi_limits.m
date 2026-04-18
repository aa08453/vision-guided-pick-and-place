% x = linspace(-35, 35, 1000);
% y = linspace(-35, 35, 1000);
x = 15; y = 0;
z = linspace(0, 45.7, 1000);

phi = atan2(z, sqrt(x.^2 + y.^2));

plot(z, phi)
axis tight;