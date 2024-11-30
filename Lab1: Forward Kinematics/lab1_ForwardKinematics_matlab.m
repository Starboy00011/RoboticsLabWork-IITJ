% Practical 1 : Force Analysis(Inverse Dynamics)

%Model Parameters
m=1;l=1;lc= 0.5;Izz=(1/3)*m*l*l;g=9.81;

%Motion Parameters
d2r=(pi/180);th0 =0*d2r ;thT= 120*d2r;T=3;step=0.01;i=1;

%For loop starts
for t=0:step:T
    time(i)=t;
    c1=(thT-th0)/T;c2=(2*pi)/T;
    thd(i)= th0 +c1 *(t-(1/c2)*sin(c2*t));   %Joint Position
    dthd(i)= c1*(1-cos(c2*t));               %Joint Velocity
    ddthd(i)= c1*(c2*sin(c2*t));             %Joint Acceleration

% Joint Torque
    tau(i)=Izz*ddthd(i)+m*g*lc*cos(thd(i));
% Animation
    x0=0;y0=0;%xe=l*cos(thd(i));ye= l*sin(thd(i));
    xe(i)=x0 +l*cos(thd(i));
    ye(i)= y0 +l*sin(thd(i));                % End Effector Position
    xx=[x0 ;xe(i)]; yy=[y0; ye(i)];
    
    figure(2)
    plot (xx,yy,xe(1:i),ye(1:i));
    
% Animation to fix the window size
    xmin=-1.5*l;xmax= 1.5*l ;ymin=-1.5*l; ymax=1.5*l;
    axis([ xmin xmax ymin ymax])
    xlabel('X(m)');ylabel('Y(m)')
    axis equal;
% End effector Velocity and accleration
    dxe(i)=-l*sin(thd(i))*dthd(i);dye(i)=l*cos(thd(i))*dthd(i);
    ddxe(i)=-l*cos(thd(i))*dthd(i)-l*sin(thd(i))*ddthd(i);
    ddye(i)=-l*sin(thd(i))*dthd(i)+l*cos(thd(i))*ddthd(i);

    i=i+1;
end
figure(1)
plot(time,thd/d2r,time,dthd/d2r,time,ddthd/d2r,time,tau)
xlabel('time(s)');ylabel('Motion')
legend('Joint angle(deg)','Joint rate(deg/s)','Joint acc(deg/s^2)','Torque(Nm)')

figure(3)
plot(time,dxe,time,dye,time,ddxe,time,ddye)
legend('dxe(m/s)','dye(m/s)','ddxe(m/s^2)','ddye(m/s^2)')