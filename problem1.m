% Contributors: Rowan LeBlanc
% Course number: ASEN 3801
% File name: Problem1
% Created: 9/8/23

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
