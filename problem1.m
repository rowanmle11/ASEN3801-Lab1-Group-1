% Contributors: Rowan LeBlanc
% Course number: ASEN 3801
% File name: Problem1
% Created: 9/8/23

function dxdt = problem1(t,xvar)
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

