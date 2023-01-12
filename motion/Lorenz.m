function dx = lorenz(t,x,g,m,r,fr,CD,A,Nt,Nf,Id,Iw,Ie,It,ntf,TMax,p,CF,CR,a,b,IZ,theta)
dx = [
% x[1] = longitudinal vel 
% x[2] = lateral vel
% x[3] = heading vel
%(-(fr*m*g)-(x(1)*x(1)*p*CD*A/2)+(TMax*(x(1)/7)*Nt*Nf*ntf/r))/(m+(((Ie+It)*Nt*Nt*Nf*Nf+Id*Nf*Nf+Iw)/(r*r)));
0;
(-x(2)*(CF+CR)/(m*x(1)))-x(3)*((CF*a-CR*b)/(m*x(1))-x(1))+(CF/m)*theta;
(-x(2)*(CF*a-CR*b)/(IZ*x(1)))-x(3)*((CF*a*a+CR*b*b)/(IZ*x(1)))+(CF*a/IZ)*theta;
];
