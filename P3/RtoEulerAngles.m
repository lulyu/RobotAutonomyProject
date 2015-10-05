function [thx, thy, thz] = RtoEulerAngles(R)
thx = atan2(R(3,2), R(3,3));
thy = atan2(-R(3,1), sqrt(R(3,2)^2+R(3,3)^2));
thz = atan2(R(2,1), R(1,1));