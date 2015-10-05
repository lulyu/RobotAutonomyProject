function [R, t] = EtoRt(E)
[U,S,V] = svd(E);
W = [0 -1 0;1 0 0;0 0 1];
t_hat = V*W*S*V';
R = U/W*V';
t = [t_hat(3,2), t_hat(1,3), t_hat(2,1)];

%% Validate the result
if t(1,1)>0
    W = inv(W);
    t_hat = V*W*S*V';
    R = U/W*V';
    t = [t_hat(3,2), t_hat(1,3), t_hat(2,1)];
end

if det(R)<0
    V = -V;
    t_hat = V*W*S*V';
    R = U/W*V';
    t = [t_hat(3,2), t_hat(1,3), t_hat(2,1)];
end