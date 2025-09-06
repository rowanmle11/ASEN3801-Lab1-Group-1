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

wind_speeds = [0,5,10,15,20,25,30,35,40,45,50];

distance_x = zeros(length(wind_speeds),1);

% (initial x coord of landing position) + wind in North direction
for i = 1:length(wind_speeds)

    W_EE = [wind_speeds(i) 0 0];
    [t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,m,g,W_EE), t_span, x_0, options);
    distance_x(i) = x(end,1);

end

figure();
plot(wind_speeds,distance_x);
xlabel('Wind Speed (m/s)');
ylabel('Distance North (m)');
title('Distance Traveled North at Various Northern Wind Speeds'); hold off;

% total distance from origin to landing position + wind in North direction

distance_all = zeros(length(wind_speeds),1);

for i = 1:length(wind_speeds)

    W_EE = [wind_speeds(i) 0 0];
    [t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,m,g,W_EE), t_span, x_0, options);
    distance_all(i) = sqrt( (x(end,1))^2 + (x(end,2))^2 );

end

figure(); 
plot(wind_speeds,distance_all);
xlabel('Wind Speed (m/s)');
ylabel('Total Distance (m)');
title('Total Distance Traveled at Various Northern Wind Speeds');

%% Problem 2e part 1

altitudes = [0,1000,1655,2000,3000,4000];
wind_speeds = [0,5,10,15,20,25,30,35,40,45,50];
distance_total = zeros(length(altitudes),length(wind_speeds));

for i = 1:length(altitudes)

    rho = stdatmo(altitudes(i));

    for j = 1:length(wind_speeds)
        
        W_EE = [wind_speeds(j),0,0];
        [t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,m,g,W_EE), t_span, x_0, options);
        distance_total(i,j) = sqrt( (x(end,1))^2 + (x(end,2))^2 );

    end

    

end

figure();
plot(wind_speeds,distance_total);
xlabel('Wind Speed (m/s)');
ylabel('Total Distance from Origin (m)');
title('Total distance Trvaeled at Various Altitudes and Wind Speeds');
legend('0 m', '1000 m','1655 m', '2000 m','3000 m', '4000 m');

%% Problem 2e part 2

distance = zeros(length(altitudes));

for i = 1:length(altitudes)
    
    rho = stdatmo(altitudes(i));
    W_EE = [30,0,0];
    [t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,m,g,W_EE), t_span, x_0, options);
    distance(i) = sqrt( (x(end,1))^2 + (x(end,2))^2 );

end

figure();
plot(altitudes,distance);
ylim([68,72]);
title('Total distance Trvaeled at Various Altitudes (Wind = 30 m/s North)');
xlabel('Altitude (m)');
ylabel('Total Distance from Origin (m)'); hold off;


%% Problem 2f
rho = stdatmo(1655);
wind_speeds = [0,5,10,15,20,25,30,35,40,45,50];
masses = linspace(0.001,.3,100); % mass in kilos
total_energy = 20; % joules
distance_2 = zeros(length(wind_speeds),length(masses));

for i = 1:length(wind_speeds)

    for j = 1:length(masses)
        
        velocity = sqrt((total_energy * 2) / masses(j));
        vel_comp = sqrt((velocity^2) / 2);
        W_EE = [0,wind_speeds(i),0];
        x_0 = [0,0,0,0,velocity,-velocity];
        [t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,masses(j),g,W_EE), t_span, x_0, options);
        distance_2(i,j) = sqrt( (x(end,1))^2 + (x(end,2))^2 );

    end

end

figure();
plot(masses,distance_2);
xlabel('Mass (kg)');
ylabel('Total Distance (m)');
title('Total Distance Traveled at Various Wind Speeds and Masses');
legend('0 m/s East','5 m/s East','10 m/s East','15 m/s East','20 m/s East', ...
    '25 m/s East','30 m/s East','35 m/s East','40 m/s East','45 m/s East', ...
    '50 m/s East');

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
