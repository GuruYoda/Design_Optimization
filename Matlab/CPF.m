function [error,circle] = CPF(x,y) 
n = numel(x); 
a = zeros(1,n); 
b = zeros(1,n); 
c = zeros(1,n); 
for i = 1:n 
    a(i) = (2/n) *sum(x)-2 *x(i); 
    b(i) = (2/n) *sum(y)-2 *y(i); 
    c(i) = x(i)^2 + y(i)^2-(1/n) *sum(x.^2 + y.^2); 
end
f1 = sum(a.^2); 
f2 = sum(b.^2); 
f3 = sum(2 *a.*c); 
f4 = sum(2 *b.*c); 
f5 = sum(2 *a.*b); 
C = [2 *f1,f5;f5,2 *f2]^-1 *[-f3;-f4]; 
Cx = C(1); % x-coordinate of center point 
Cy = C(2); % y-coordinate of center point 
R2 = zeros(1,n); 
for i = 1:n 
    R2(i) = (x(i)-Cx) ^ 2 + (y(i)-Cy) ^ 2;
end
error = sum(((R2-mean(R2))/mean(R2)).^2); %dimensionless error 
R = mean(sqrt(R2)); % the radius of best fitting circle 
circle = [Cx Cy R];

