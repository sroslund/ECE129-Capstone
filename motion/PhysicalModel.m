clear all


% Lorenz parameters my car
g = 9.81;
m = 544; % mass Kg
r = 0.28; % tire radius m
fr = 0.02; % rolling resistance
CD = 0.3; % air drag
A = 2.1; % area m^2
Nt = 3.1; % transmission gear ratio
Nf = 5.8; % differential gear ratio
Id = 0.06; % driveshaft inertia Kg*m^2
Iw = 0.45; % wheel and axle inertia Kg*m^2
Ie = 0.002; % engine inertia Kg*m^2
It = 0.01; % transmission inertia Kg*m^2
ntf = .85; % drivetrain efficiency
TMax = 11.6; % original 11.6 maximum engine torque N*m
p = 1.22; % mass density of air Kg/m^3
CF = 60000; % front tire cornering stiffness N / rad
CR = 30000; % rear tire cornering stiffness N / rad
a = 0.85; % distance from CG to front axle m
b = 0.9; % distance from CD to rear axle m
IZ = 416; % car moment of inertia around z axis Kg * m^2


% initial conditions
x0 = [0.5;0;0]; % [longitudinal vel;lateral vel;heading vel]
theta = 0.523; % rad angle of turned wheels

% compute trajectory
dt = 0.01;
tspan = 0:dt:60; 

[t,x] = ode45(@(t,x)lorenz(t,x,g,m,r,fr,CD,A,Nt,Nf,Id,Iw,Ie,It,ntf,TMax,p,CF,CR,a,b,IZ,theta),tspan,x0);

for c = 1:size(tspan,2)
    earthXvel(c,1) = 0.5*cos(dt*trapz(x(1:c,3)))-x(c,2)*sin(dt*trapz(x(1:c,3)));
    earthYvel(c,1) = 0.5*sin(dt*trapz(x(1:c,3)))+x(c,2)*cos(dt*trapz(x(1:c,3)));

    earthXpos(c,1) = dt*trapz(earthXvel(1:c,1));
    earthYpos(c,1) = dt*trapz(earthYvel(1:c,1));
end

plot(earthXpos,earthYpos);
modeledRadius = (max(earthXpos)-min(earthXpos))/2
actualRadius = (.85+.9)/sin(theta)


% NO SLIP ANGLE RADIUS
% R = (a+b)/sin(theta)
% R = (.85+.9)/sin(0.523)
