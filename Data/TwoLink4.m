function Xdot = TwoLink4(t,X)
Amp1=0.1; Amp2=0.1; per=1;
kp=80; kv=20;
m1=1; m2=1; a1=1; a2=1; g=9.8;

% COMPUTE DESIRED TRAJECTORY qd(t), qdp(t), qdpp(t)
fact=pi/per;
qd(1) = Amp1*sin(fact*t);
qd(2) = Amp2*cos(fact*t);
qdp(1) = Amp1*fact*cos(fact*t);
qdp(2) = -Amp2*fact*sin(fact*t);
qdpp(1)= -Amp1*(fact^2)*sin(fact*t);
qdpp(2)= -Amp2*(fact^2)*cos(fact*t);

% COMPUTED-TORQUE CONTROLLER SUBROUTINE
% COMPUTE TRACKING ERRORS;
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
s1 = qdpp(1) + kv*ep(1) + kp*e(1);
s2 = qdpp(2) + kv*ep(2) + kp*e(2);
f1 = m11*s1 + m12*s2 + N1;
f2 = m21*s1 + m22*s2 + N2;

% ROBOT ARM DYNAMICS:
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
