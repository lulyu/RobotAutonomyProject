function [X, err] = triangulate_diy(P1,x1,P2,x2)
X = zeros(3,size(x1,2));
for i=1:size(x1,2)
    A(1,:)=[bsxfun(@minus,x1(2,i)*P1(3,:),P1(2,:))];
    A(2,:)=[bsxfun(@minus,P1(1,:),x1(1,i)*P1(3,:))];
    A(3,:)=[bsxfun(@minus,x2(2,i)*P2(3,:),P2(2,:))];
    A(4,:)=[bsxfun(@minus,P2(1,:),x2(1,i)*P2(3,:))];

    [~,~,V] = svd(A'*A);
    Xi = V(:,size(V,2));
    err = norm(A*Xi);
    X(:,i) = Xi(1:3)/Xi(4);
end
