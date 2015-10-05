function [R,t,P,err] = reproject(x, X, norm1, norm2, K)
norm1 = 1; norm2 = 1;
x = bsxfun(@rdivide, x, norm1);
X = bsxfun(@rdivide, X, norm2);

%----- Make Matrix A --------------------------------------------
A = zeros(2*size(x,1),12);
for i=1:size(x,1)
    A(2*(i-1)+1,:)=[X(:,i)' 1 0 0 0 0 -x(i,1)*[X(:,i)' 1]];
    A(2*i,:)=[0 0 0 0 X(:,i)' 1 -x(i,2)*[X(:,i)' 1]];
end

% [v,d] = eig(U'*U);
% f = v(:,1);
% F = reshape(f,3,3);

[~,S,V] = svd(A'*A);
l = V(:,size(V,2));
P(1,:)=l(1:4)/norm1; P(2,:)=l(5:8)/norm1; P(3,:)=l(9:12)/(norm1*norm2);

[~,~,v] = svd(P);
c = -bsxfun(@rdivide,v(1:3,4),v(4,4));
M = P/[diag([1,1,1]) c];
[q,r] = qr(M);
N = diag([sign(q(1,1)) sign(q(2,2)) sign(q(3,3))]);
R = q*N;

% Find closest rotation solution...
% [thx, thy, thz] = RtoEulerAngles(R);
% thx = unroll(thx); thy = unroll(thy); thz = unroll(thz); 
% R = ZYXToR([thx thy thz]);

t = R*c;%*(abs(K/r(1,1)));
% R = P(:,1:3);
% P = bsxfun(@rdivide, P, sign(det(R))*det(abs(R))^(1/3));
% R = P(:,1:3);
% t = P(:,4);

err = sum(abs(A*l));
err;

