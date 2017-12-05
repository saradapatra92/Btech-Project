function Xdot = TwoLink2(t,X)
% X(1)= theta1
% X(2)= theta2
% X(3) = (d/dt)theta1
% X(4) = (d/dt)theta2
% X(5) = torque input 1 (Only for Output purposes)
% X(6) = torque input 2 (Only for Output purposes)
% X(7) = tracking error 1 (Only for Output purposes)
% X(8) = tracking error 2 (Only for Output purposes)
% Constant values for the desired trajectory:
function Xdot = slidingMode(t,X)
syms V 'gamma' b 'tau' s 'eta'
Amp1=0.1; Amp2=0.1; per=1;
% Constant values of the controller:
kp=80; kv=20;
% Constant values of the Robot:
m1=1; m2=1; a1=1; a2=1; g=9.8;

% PART 1) COMPUTE DESIRED TRAJECTORY qd(t), qdp(t), qdpp(t)

fact=pi/per;
qd(1) = Amp1*sin(fact*t);
qd(2) = Amp2*cos(fact*t);
qdp(1) = Amp1*fact*cos(fact*t);
qdp(2) = -Amp2*fact*sin(fact*t);
qdpp(1)= -Amp1*(fact^2)*sin(fact*t);
qdpp(2)= -Amp2*(fact^2)*cos(fact*t);

% PART 2) COMPUTED-TORQUE CONTROLLER SUBROUTINE
% COMPUTE TRACKING ERRORS:
e(1) = qd(1)-X(1);
e(2) = qd(2)-X(2);
ep(1) = qdp(1)-X(3);
ep(2) = qdp(2)-X(4);

% COMPUTATION OF M(q)
m11=(m1+m2)*a1^2+m2*a2^2+2*m2*a1*a2*cos(X(2));
m12=m2*a2^2+m2*a1*a2*cos(X(2));
m21=m12;
m22=m2*a2^2;

% COMPUTATION OF N(q,qp)
N1 = -m2*a1*a2*(2*X(3)*X(4)+X(4)^2)*sin(X(2));
N2 = m2*a1*a2*X(3)^2*sin(X(2));
N1 = N1 + (m1+m2)*g*a1*cos(X(1))+m2*g*a2*cos(X(1)+X(2));
N2 = N2 + m2*g*a2*cos(X(1)+X(2));

% COMPUTATION OF CONTROL TORQUES
V=5*dig(2.5,0.5,1.5,5,5,5);
r=e(p)+V*e;
M*r(p)=-cr+f(x)+bd-b;
f(x)=M(q)*(qdpp+V*e(p)+c(qd)*(qdp+V*e));
f1 = m11*s1 + m12*s2 + N1;
f2 = m21*s1 + m22*s2 + N2;

% PART 3) ROBOT ARM DYNAMICS:
Det = (m11*m22)-(m12*m21);

% Inverse Matrix
Im11=m22/Det;
Im12=-m12/Det;
Im21=-m21/Det;
Im22=m11/Det;

Xdot=[X(3);
X(4);
(Im11*(f1-N1)+Im12*(f2-N2));
(Im21*(f1-N1)+Im22*(f2-N2));
(f1-X(5))/(.01);
(f2-X(6))/(.01);
(e(1)-X(7))/(.01);
(e(2)-X(8))/(.01)
];
