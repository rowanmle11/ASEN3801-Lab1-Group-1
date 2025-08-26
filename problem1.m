% Contributors: Rowan LeBlanc
% Course number: ASEN 3801
% File name: Problem1
% Created: 9/8/23

clear
clc
close all

%% Problem 1a

init_cond = [1;1;1;1];

t_span = [0;20];

[t, sol] = ode45(@problem1a, t_span, init_cond);

figure
labels = {'w (n.d.)','x (n.d.)','y (n.d.)','z (n.d.)'};
for i = 1:4
    subplot(4,1,i)
    plot(t, sol(:,i),'LineWidth',1.5)
    ylabel(labels{i})
    if i==4
        xlabel('Time (n.d.)')
    end
    grid on
end
sgtitle('Problem 1a: Evolution of w, x, y, z')

%% Problem 1b

tols = [1e-2; 1e-4; 1e-6; 1e-8; 1e-10; 1e-12];

opts_ref = odeset('RelTol',1e-12,'AbsTol',1e-12);
[t_ref,y_ref] = ode45(@problem1a,t_span,init_cond,opts_ref);
yR = y_ref(end,:); % reference final values

% Preallocate error table
errors = zeros(4,length(tols)-1);

for i=1:length(tols)-1
    opts = odeset('RelTol',tols(i),'AbsTol',tols(i));
    [t,y] = ode45(@problem1a,t_span,init_cond,opts);
    errors(:,i) = abs(y(end,:) - yR);
end

% Display table
rowNames = {'|w - w_R|','|x - x_R|','|y - y_R|','|z - z_R|'};
errorTable = array2table(errors,'RowNames',rowNames,...
    'VariableNames',{'1e-2','1e-4','1e-6','1e-8','1e-10'});

disp('Problem 1b: Error table relative to tol=1e-12 reference')
disp(errorTable)

%% Functions

function dxdt = problem1a(t,xvar)
    w=xvar(1);
    x=xvar(2);
    y=xvar(3);
    z=xvar(4);

    dxdt=zeros(4,1);
    dxdt(1)=(-9.*w)+y;
    dxdt(2)=(4.*w.*x.*y)-(x.^2);
    dxdt(3)=(2.*w)-x-(2.*z);
    dxdt(4)=(x.*y)-(y.^2)-(3.*z.^3);
end
