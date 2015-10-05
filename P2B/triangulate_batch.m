function [X] = triangulate_batch(x1,x2,P1,P2)
X = zeros(4,size(x1,1));
err = zeros(1,size(x1,1));
for i = 1:size(x1,1)
    [X(:,i)] = triangulate(P1,x1(i,:)',P2,x2(i,:)');
end
% X = bsxfun(@rdivide,X,X(4,:));

% Eliminate pairs with great error
