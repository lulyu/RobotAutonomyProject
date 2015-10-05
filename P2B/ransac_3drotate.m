function [R, t, inliers, ifFound] = ransac_3drotate(X1, X2)
Num = 1000; % times of RANSAC
Threshold = 0.20; % the threshold to determine inliers

%--- initialize all output variables --------------
max_num_inliers = 0;
R = zeros(3);
t = zeros(3,1);
inliers = zeros(1,size(X1,2));
ifFound = 0;
err = 1000;

for i=1:Num
    %--- calculate a RANSAC R/t ------------------------
    all_rand = randperm(size(X1,2));
    ind_rand = all_rand(1:4);
    
    [T, Eps] = estimateRigidTransform(X1(:,ind_rand), X2(:,ind_rand));
    R_rand = T(1:3,1:3);
    %R_rand = sign(det(R_rand))*R_rand;
    t_rand = T(1:3,4);
         
    num_inliers = 0;
    err_rand = 0;
    d = zeros(size(X1,2),1);
    for k=1:size(X1,2)
        d(k) = norm([X2(:,k);1]-T*[X1(:,k);1]);
        if abs(d(k))<Threshold % Found a inlier!!!
            num_inliers = num_inliers + 1;
            inliers_rand(1,num_inliers) = k;
            err_rand = err_rand + abs(d(k));
        end
    end

    if (num_inliers>max_num_inliers)&&(num_inliers>0) % Found a better F!!!
        max_num_inliers = num_inliers;
        R = R_rand;
        t = t_rand;
        err = err_rand/num_inliers;
        inliers = inliers_rand;
        ifFound = 1;
%         [T, Eps] = estimateRigidTransform(X1(:,inliers), X2(:,inliers));
%         R = T(1:3,1:3);
%         t = T(1:3,4);
    elseif (num_inliers==max_num_inliers)&&(num_inliers>3)
        if err_rand/num_inliers < err 
            max_num_inliers = num_inliers;
            R = R_rand;
            t = t_rand;
            err = err_rand/num_inliers;
            inliers = inliers_rand;
            ifFound = 1;
            [T, Eps] = estimateRigidTransform(X1(:,inliers), X2(:,inliers));
            R = T(1:3,1:3);
            t = T(1:3,4);
        end
    end
end

inliers = inliers(find(inliers));

