%Contributors: Rowan LeBlanc, Ian Jaggi,
% Course number: ASEN 3801
% File name: problem2.m
% Created: 09/02/2025

clear
clc
close all

%% Problem 2c

m = 0.05; %kg
d = 0.02; %m
g = 9.8;
A = pi*(d/2)^2;
Cd = 0.6;
rho = stdatmo(1655);

t_span = [0 20];

x_0 = [0 0 0 0 20 -20]; W_EE = [0 0 0];

options = odeset('Reltol',1e-8,'AbsTol',1e-8);
[t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,m,g,W_EE), t_span, x_0, options);

% original plot
figure
plot3(x(:,1),x(:,2),-x(:,3))
grid on
xlabel('X'); ylabel('Y'); zlabel('Z')

%% Problem 2d

% (initial x coord of landing position) + wind in North direction
W_EE = [10 0 0];
[t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,m,g,W_EE), t_span, x_0, options);

figure
plot3(x(:,1),x(:,2),-x(:,3))
grid on
xlabel('X'); ylabel('Y'); zlabel('Z')

% total distance from origin to landing position + wind in North direction
W_EE = [20 0 0];
[t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,m,g,W_EE), t_span, x_0, options);

figure
plot3(x(:,1),x(:,2),-x(:,3))
grid on
xlabel('X'); ylabel('Y'); zlabel('Z')

%% Function (Problem 2a)

function xdot = objectEOM(t,x,rho,Cd,A,m,g,W_EE)
    V_EE = [x(4) x(5) x(6)];
    V_E = V_EE - W_EE;
    V_A = norm(V_E);
    
    drag = 0.5*rho*(V_A^2)*A*Cd;
    
    a_i = (-drag/m)*(V_E(1)/V_A); 
    a_j = (-drag/m)*(V_E(2)/V_A);
    a_k = g + ((-drag)/m)*(V_E(3)/V_A);
    
    if x(3) > 0
        xdot = 0;
    else 
        xdot = [V_EE(1); V_EE(2); V_EE(3); a_i; a_j; a_k]; 
    end
end
