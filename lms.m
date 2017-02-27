%LMS Function
function [h,y,E] = lms(x,d,delta,N)
M = length(x); %length of the input sequence
y = zeros(1,M); %output signal
h = zeros(1,N); %impulse response
for n = N:M
    x1 = x(n:-1:n-N+1); %summation of the past input values 
    y(n) = h*transpose(x1); %output signal computation
    e = d(n) - y(n); %error 
    h = h + delta*e*x1; %impluse response for samples
    E(n) = e; %storing error in E
end 
end 
    
