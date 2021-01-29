function [error,linkage] = objfun(X) 
Xd = [-1.7476,-1.6409,-1.5302,-1.4158,-1.2983,-1.1786,-1.0583,-0.9389,-0.8223,-0.7102,-0.6045,-0.5067,-0.4182,-0.34,-0.2728,-0.217,-0.1729,-0.1403,-0.1188,-0.1078,-0.1067,-0.1146,-0.1304,-0.1532,-0.1818,-0.215,-0.2517,-0.2906,-0.3306,-0.3706,-0.4094,-0.446,-0.4796,-0.5092,-0.534,-0.5537,-0.5677,-0.5762,-0.5794,-0.5783,-0.5749,-0.5721,-0.5751,-0.5913,-0.63,-0.7003,-0.8066,-0.9461,-1.1105,-1.2908,-1.4789,-1.6691,-1.8571,-2.0399,-2.2152,-2.3813,-2.537,-2.6813,-2.8133,-2.537,-3.0386,-3.1311,-3.2099,-3.275,-3.3265,-3.3644,-3.3891,-3.4009,-3.4004,-3.3879,-3.3642,-3.3301,-3.2861,-3.2334,-3.1726,-3.1049,-3.0311,-2.9522,-2.8692,-2.7831,-2.6946,-2.6045,-2.5133,-2.4215,-2.3291,-2.2362,-2.1425,-2.0474,-1.9503,-1.8506];
Yd = [3.5356,3.9034,4.2837,4.6713,5.0613,5.4487,5.8288,6.1974,6.5508,6.8861,7.2007,7.493,7.7615,8.0056,8.2247,8.4186,8.5874,8.7314,8.8509,8.9463,9.0181,9.067,9.0934,9.0979,9.0813,9.0442,8.9872,8.911,8.8165,8.7043,8.5753,8.4303,8.2702,8.096,7.9088,7.7096,7.4998,7.2807,7.0539,6.8216,6.5859,6.3502,6.1183,5.8956,5.6877,5.4992,5.3313,5.1801,5.0394,4.9028,4.7658,4.6253,4.4798,4.3283,4.1707,4.0074,3.8387,3.6655,3.4886,3.309,3.1277,2.9461,2.7652,2.5863,2.4107,2.2398,2.075,1.9176,1.7691,1.6309,1.5045,1.3913,1.2929,1.2107,1.1463,1.1011,1.0768,1.0747,1.0964,1.1434,1.217,1.3184,1.4487,1.6087,1.799,2.0195,2.2698,2.5488,2.8546,3.1846];
xA = X(1); 
yA = X(2); 
r3 = X(3); 
beta = X(4); 
n = numel(Xd); 
R = zeros(1,n); 
for i = 1:n 
    R(i) = sqrt((Xd(i)-xA) ^ 2 + (Yd(i)-yA) ^ 2); 
end
[Rmax,max_index] = max(R); 
[Rmin,min_index] = min(R); 
if inpolygon(xA,yA,Xd,Yd) 
    r2 = (Rmax + Rmin)/2;
    r5 = (Rmax-Rmin)/2; 
else
    r2 = (Rmax-Rmin)/2; 
    r5 = (Rmax + Rmin)/2; 
end
xC1 = zeros(1,n); % x-coordinate of point C in trajectory 01
yC1 = zeros(1,n); % y-coordinate of point C in trajectory 01 
xC2 = zeros(1,n); % x-coordinate of point C in trajectory 02 
yC2 = zeros(1,n); % y-coordinate of point C in trajectory 02 
for i = 1:n 
    if (i < max_index && i > min_index) || (i > max_index && i < min_index) 
        % trajectory 01 
        theta2m1 = atan2(Yd(i)-yA,Xd(i)-xA) + acos((r2 ^ 2 + R(i) ^ 2-r5 ^ 2)/(2 *r2 *R(i))); 
        theta5m1 = atan2(Yd(i)-yA-r2 *sin(theta2m1),Xd(i)-xA-r2 *cos(theta2m1)); 
        theta3m1 = theta5m1-beta; 
        xC1(i) = xA + r2 *cos(theta2m1) + r3 *cos(theta3m1); 
        yC1(i) = yA + r2 *sin(theta2m1) + r3 *sin(theta3m1); 
        % trajectory 02 
        theta2m2 = atan2(Yd(i)-yA,Xd(i)-xA)-acos((r2 ^ 2 + R(i) ^ 2-r5 ^ 2)/(2 *r2 *R(i))); 
        theta5m2 = atan2(Yd(i)-yA-r2 *sin(theta2m2),Xd(i)-xA-r2 *cos(theta2m2)); 
        theta3m2 = theta5m2-beta; 
        xC2(i) = xA + r2 *cos(theta2m2) + r3 *cos(theta3m2); 
        yC2(i) = yA + r2 *sin(theta2m2) + r3 *sin(theta3m2);
        
    elseif i == max_index 
        theta2 = atan2(Yd(i)-yA,Xd(i)-xA); 
        theta5 = theta2; 
        theta3 = theta5-beta; 
        xC1(i) = xA + r2 *cos(theta2) + r3 *cos(theta3); 
        yC1(i) = yA + r2 *sin(theta2) + r3 *sin(theta3); 
        xC2(i) = xC1(i); 
        yC2(i) = yC1(i);
        
    elseif i == min_index 
        if inpolygon(xA,yA,Xd,Yd) 
            theta2 = atan2(Yd(i)-yA,Xd(i)-xA); 
            theta5 = pi + theta2; 
            theta3 = theta5-beta; 
            xC1(i) = xA + r2 *cos(theta2) + r3 *cos(theta3); 
            yC1(i) = yA + r2 *sin(theta2) + r3 *sin(theta3); 
            xC2(i) = xC1(i); 
            yC2(i) = yC1(i);
            
        else
            theta2 = pi + atan2(Yd(i)-yA,Xd(i)-xA); 
            theta5 = atan2(Yd(i)-yA,Xd(i)-xA); 
            theta3 = theta5-beta; 
            xC1(i) = xA + r2 *cos(theta2) + r3 *cos(theta3); 
            yC1(i) = yA + r2 *sin(theta2) + r3 *sin(theta3); 
            xC2(i) = xC1(i); 
            yC2(i) = yC1(i); 
        end
        
    else
        % trajectory 02 
        theta2m1 = atan2(Yd(i)-yA,Xd(i)-xA) + acos((r2 ^ 2 + R(i)^2-r5 ^ 2)/(2 *r2 *R(i))); 
        theta5m1 = atan2(Yd(i)-yA-r2 *sin(theta2m1),Xd(i)-xA-r2 *cos(theta2m1)); 
        theta3m1 = theta5m1-beta; 
        xC2(i) = xA + r2 *cos(theta2m1) + r3 *cos(theta3m1); 
        yC2(i) = yA + r2 *sin(theta2m1) + r3 *sin(theta3m1); 
        % trajectory 01 
        theta2m2 = atan2(Yd(i)-yA,Xd(i)-xA)-acos((r2 ^ 2 + R(i) ^ 2-r5 ^ 2)/(2 *r2 *R(i))); 
        theta5m2 = atan2(Yd(i)-yA-r2 *sin(theta2m2),Xd(i)-xA-r2 *cos(theta2m2)); 
        theta3m2 = theta5m2-beta; 
        xC1(i) = xA + r2 *cos(theta2m2) + r3 *cos(theta3m2);
        yC1(i) = yA + r2 *sin(theta2m2) + r3 *sin(theta3m2); 
    end
    
end
[C1CPF,circle1] = CPF(xC1,yC1); %CPF error for trajectory 01 
[C2CPF,circle2] = CPF(xC2,yC2); %CPF error for trajectory 02 
[error,index] = min([C1CPF,C2CPF]); 
if index == 1 
    % if trajectory 01 is more similar to a circular curve 
    xD = circle1(1); 
    yD = circle1(2); 
    r1 = sqrt((xA-xD) ^ 2 + (yA-yD) ^ 2); 
    r4 = circle1(3); 
    alpha = atan2(yD-yA,xD-xA); 
else
    % if trajectory 02 is more similar to a circular curve 
    xD = circle2(1); 
    yD = circle2(2); 
    r1 = sqrt((xA-xD) ^ 2 + (yA-yD) ^ 2); 
    r4 = circle2(3); 
    alpha = atan2(yD-yA,xD-xA); 
end
linkage = [r1,r2,r3,r4,r5,beta,xA,yA,alpha]; 
s = min(linkage(1:4)); l = max(linkage(1:4)); 
pq = sum(linkage(1:4))-(s + l); 
error = error + (s + l >= pq) *1000 + (linkage(2) ~= s) *1000;