function Xdot = slidingMode(t,X)
syms V 'gamma' b 'tau' s 'eta'
Amp1=0.1;Amp2=0.1;
m1=1; m2=1; a1=1; a2=1; g=9.8;
per=1;q=2;

fact=pi/per;
qd(1) = Amp1*sin(fact*t);
qd(2) = Amp2*cos(fact*t);
qdp(1) = Amp1*fact*cos(fact*t);
qdp(2) = -Amp2*fact*sin(fact*t);
qdpp(1)= -Amp1*(fact^2)*sin(fact*t);
qdpp(2)= -Amp2*(fact^2)*cos(fact*t);

e=qd-q;
V=5*dig(2.5,0.5,1.5,5,5,5);

e(1) = qd(1)-X(1);
e(2) = qd(2)-X(2);
ep(1) = qdp(1)-X(3);
ep(2) = qdp(2)-X(4);

r=e(p)+V*e;
kv=40*dig(8,1,1,5,1,1,0.8);

m11=(m1+m2)*a1^2+m2*a2^2+2*m2*a1*a2*cos(X(2));
m12=m2*a2^2+m2*a1*a2*cos(X(2));
m21=m12;
m22=m2*a2^2;

f(x)=M(q)*(qdpp+V*e(p)+c(qd)*(qdp+V*e));
r(p)=((-Cr+f(x))+bd-b)/M;
kv = (kv*e(p)+kv*V*e)/r;
s=0.1;
u(t)=-(f(x)+s)*sign(r);
%b=f(x)+kv*r-u(t);
Xdot = f(x)+kv*r-u(t);