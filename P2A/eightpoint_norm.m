function F = eightpoint_norm(pts1, pts2, normalization_constant)
%----- Normalization --------------------------------------------
pts1 = bsxfun(@rdivide, pts1, normalization_constant);
pts2 = bsxfun(@rdivide, pts2, normalization_constant);

%----- Make Matrix U --------------------------------------------
U = zeros(size(pts1,2),9);
for i=1:size(pts1,2)
    U(i,1:3)=[pts1(1,i)*pts2(1,i) pts1(1,i)*pts2(2,i) pts1(1,i)];
    U(i,4:6)=[pts1(2,i)*pts2(1,i) pts1(2,i)*pts2(2,i) pts1(2,i)];
    U(i,7:9)=[pts2(1,i), pts2(2,i), 1];
end

% [v,d] = eig(U'*U);
% f = v(:,1);
% F = reshape(f,3,3);

[~,S,V] = svd(U'*U);
f = V(:,size(V,2));
F = reshape(f,3,3);

%----- Retain singularity ---------------------------------------
[U,S,V] = svd(F);
S(3,3) = 0;
F = U*S*V';

T = [1/normalization_constant, 0, 0; 0, 1/normalization_constant, 0; 0, 0, 1];
F = T'*F*T;
