f = 1000;
fs = 8000;
t = 1:1:100;
x  = 4.*sin(2.*pi*f/fs*t);
%plot(x);
x1 =  4.*sin(2*pi*f/fs*t + pi);
%plot(x1);
 y = x + x1;
 plot(y);
