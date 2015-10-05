function [E] = ZYXtoE(angles)
phi = angles(1);
theta = angles(2);
psi = angles(3);

E(1,1)=1;
E(1,2)=sin(phi)*tan(theta);
E(1,3)=cos(phi)*tan(theta);
E(2,1)=0;
E(2,2)=cos(phi);
E(2,3)=-sin(phi);
E(3,1)=0;
E(3,2)=sin(phi)*sec(theta);
E(3,3)=cos(phi)*sec(theta);