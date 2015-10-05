function [R, t, inliers, ifFound] = ransac_reproject(x, X, K)
Num = 300; % times of RANSAC
Threshold = 0.05; % the threshold to determine inliers

%--- initialize all output variables --------------
max_num_inliers = 0;
R = zeros(3);
t = zeros(3,1);
inliers = zeros(1,size(x,1));
ifFound = 0;

for i=1:Num
    %--- calculate a RANSAC R/t ------------------------
    if size(x,1)>6
    all_rand = randperm(size(x,1));
    ind_rand = all_rand(1:6);
    [R_rand,t_rand,P_rand, err] = reproject(x(ind_rand,:), X(:,ind_rand),max(max(x)),max(max(X)),K(1,1));
    else
        R=[]; t=[]; inliers=[]; ifFound=0;
        return;
    end
         
    num_inliers = 0;
    d = zeros(size(x,1),1);
    %P = K*[R_rand t_rand];
    P = P_rand;
    for k=1:size(x,1)
        A(1,:)=[X(:,k)' 1 0 0 0 0 -x(k,1)*[X(:,k)' 1]];
        A(2,:)=[0 0 0 0 X(:,k)' 1 -x(k,2)*[X(:,k)' 1]];
        l = [P(1,:) P(2,:) P(3,:)]';
        d(k) = sum(abs(A*l));
        if abs(d(k))<Threshold % Found a inlier!!!
            num_inliers = num_inliers + 1;
            inliers_rand(1,num_inliers) = k;
        end
    end

    if num_inliers>max_num_inliers % Found a better F!!!
        max_num_inliers = num_inliers;
        R = R_rand;
        t = t_rand;
        P = P_rand;
        inliers = inliers_rand;
        ifFound = 1;
    end
    i;
end

% M = inv(K)*P;
% M = M/ nthroot(det(M(:,1:3)),3);
% R = M(:,1:3);
% t = M(:,4);
inliers = inliers(find(inliers));

