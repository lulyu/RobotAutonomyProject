function [X1_n, X2_n, ind_accepted] = rejectOutliers(X1, X2)
% X1 X2 should be 2*N or 3*N...
ave_err = sum(abs(bsxfun(@minus, X1, X2)),2)/size(X1,2);
ind_accepted = [];
k = 1;
for i=1:size(X1,2)
    err = bsxfun(@minus, X1(:,i), X2(:,i));
    if size(X1,1)==3
        if (err(1)>ave_err(1)||err(2)>ave_err(2)||err(3)>ave_err(3))
        else
            ind_accepted(k) = i;
            k = k+1;
        end
    elseif size(X1,1)==2
        if (err(1)>ave_err(1)||err(2)>ave_err(2))
        else
            ind_accepted(k) = i;
            k = k+1;
        end
    end
end

X1_n = X1(:,ind_accepted);
X2_n = X2(:,ind_accepted);