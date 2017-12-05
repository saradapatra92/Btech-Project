 tf=10;
 x0=[0.1;0;0;0;0;0;0;0];
Amp1=0.1; Amp2=0.1;
L1=1;
L2=1;
t0=0;
tspan=linspace(t0,tf,1000); 

[t,X]=ode45('TwoLink4',tspan,x0);

figure(1)
plot(t,X(:,1),'b',t,X(:,2),'r'), grid on 
ylabel('\theta angles')
xlabel('Time (seconds)')
title('Two Link Planar Robot: \theta1 in blue, \theta2 in red')

figure(2)
subplot(2,1,1)
plot(t,X(:,3),'b'), grid on
ylabel('(d/dt) \theta1')
xlabel('time')
title('theta velocities for the Two Link Planar Robot')
subplot(2,1,2)
plot(t,X(:,4),'r'), grid on 
ylabel('(d/dt) \theta2')
xlabel('time')

figure(3)
subplot(2,1,1)
plot(X(:,1),X(:,3),'b'), grid on
title('PHASE PLOTS')
ylabel('(d/dt) \theta1')
xlabel(' \theta1')
subplot(2,1,2)
plot(X(:,2),X(:,4),'r'), grid on
ylabel('(d/dt) \theta2')
xlabel('\theta2')

% Plot of the desired trajectory:
figure(4)
qd1 = Amp1*sin(pi*t);
qd2 = Amp2*cos(pi*t);
xd=L1*cos(qd1)+L2*cos(qd1+qd2);
yd=L1*sin(qd1)+L2*sin(qd1+qd2);
plot3(xd,t,yd,'r'), grid on % time domain plots
axis([-2,2,0,10,-1,1])
view([2,-.8,1])
title('DESIRED displacement of the Tool in (X,Y) in blue')
ylabel('( time )')
xlabel('( X )')
zlabel('( Y )')

% Plot of the REAL trajectory:
figure(5)
x=L1*cos(X(:,1))+L2*cos(X(:,1)+X(:,2));
y=L1*sin(X(:,1))+L2*sin(X(:,1)+X(:,2));
plot3(xd,t,yd,'-r'), grid on % time domain plots
hold on
plot3(x,t,y,'b') % time domain plots
axis([-2,2,0,10,-1,1])
view([2,-.8,1])
title('Real displacement of the Tool in (X,Y) in blue')
ylabel('( time )')
xlabel('( X )')
zlabel('( Y )')

% Also, Torque Forces can be plotted using the following commands:
figure(6)
grid on
plot(t,X(:,5),'b'), grid on % time domain plots
title('TORQUE INPUTS f1 in blue, f2 in red')
ylabel('( time ) ')
xlabel('( Forces )')
hold on
plot(t,X(:,6),'r'), grid on % time domain plots

% Also, Tracking error can be plotted using the following commands:
figure(7)
grid on
plot(t,X(:,7),'b'), grid on % traking error of theta1
title('TRACKING ERROR e1 in blue, e2 in red')
xlabel('( time ) ')
ylabel('( X )')
hold on
plot(t,X(:,8),'r'), grid on % traking error of theta2