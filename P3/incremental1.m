function [M2, incR, incT] = incremental1(LeftImg, LeftImgNext, RightImg, RightImgNext)
%% To Find incremental translation and rotation between each successive left image pair
LeftImg     = rgb2gray(LeftImg);
LeftImgNext = rgb2gray(LeftImgNext);
RightImg     = rgb2gray(RightImg);
RightImgNext = rgb2gray(RightImgNext);

points1     = detectHarrisFeatures(LeftImg);
points1Next = detectHarrisFeatures(LeftImgNext);
points2     = detectHarrisFeatures(RightImg);
points2Next = detectHarrisFeatures(RightImgNext);


[features1, valid_points1]          = extractFeatures(LeftImg, points1);
[features1Next, valid_points1Next]  = extractFeatures(LeftImgNext, points1Next);
[features2, valid_points2]          = extractFeatures(RightImg, points2);
[features2Next, valid_points2Next]  = extractFeatures(RightImgNext, points2Next);


indexPairs             = matchFeatures(features1, features2);
indexPairsNext         = matchFeatures(features1Next, features2Next);
indexPairsBetweenLeft  = matchFeatures(features1, features1Next);

Left = zeros(1, length(indexPairs));
LeftNext = zeros(1, length(indexPairs));

%% Finding Matched features in Left, Right, LeftNext, RightNext
for i = 1: length(indexPairs)
    currLeft = indexPairs(i,1);
    currLeftBetween = find(indexPairsBetweenLeft(:,1) == currLeft);
    if(~isempty(currLeftBetween))
        currLeftNext = indexPairsBetweenLeft(currLeftBetween, 2);
        pair = find(indexPairsNext(:,1) == currLeftNext);
        if(~isempty(pair))
            Left(i) = i;
            LeftNext(i) = pair;
        end
    end

end

Left = Left(Left~=0);
LeftNext = LeftNext(LeftNext~=0);
% % l = features1.Features;
% % lNext = features1Next.Features;
% % for i = 1 : length(Left)
% %     fprintf('%f',sum(l(indexPairs(Left(i),1),:)));
% %     fprintf('\t');
% %     fprintf('%f',sum(lNext(indexPairsNext(LeftNext(i),1),:)));
% %     fprintf('\n');
% % end

matchedPoints1 = valid_points1(indexPairs(Left,1));
matchedPoints2 = valid_points2(indexPairs(Left,2));
matchedPoints1Next = valid_points1Next(indexPairsNext(LeftNext,1));
matchedPoints2Next = valid_points2Next(indexPairsNext(LeftNext,2));

% showMatchedFeatures(LeftImg, LeftImgNext, matchedPoints1, matchedPoints1Next);

%% Camera Intrinsic Matrix
K = [164.255034407511 0.0 214.523999214172; 0.0 164.255034407511 119.433252334595; 0.0 0.0 1.0];
f = 164.255034407511;

%% Stereo Baseline
b = 0.1621;  %%(in m)

%% 3D point cloud generation at current time step

% d = matchedPoints1.Location(:,1)-matchedPoints2.Location(:,1);
% X(3,i) = (f*b)/d;
% X(1,i) = matchedPoints1.Location(i,1)*(X(3,i)/f);
% X(2,i) = matchedPoints1.Location(i,2)*(X(3,i)/f);
X=zeros(3,matchedPoints1.Count);
for i = 1 :matchedPoints1.Count
    d = matchedPoints1.Location(i,1)-matchedPoints2.Location(i,1);
    X(3,i) = (f*b)/d;
    X(1,i) = matchedPoints1.Location(i,1)*(X(3,i)/f);
    X(2,i) = matchedPoints1.Location(i,2)*(X(3,i)/f);
end
 %scatter3(X(1,:),X(2,:),X(3,:))

%% 3D point cloud generation at next time step
XNext=zeros(3,matchedPoints1Next.Count);
for i = 1 :matchedPoints1Next.Count
    d = matchedPoints1Next.Location(i,1)-matchedPoints2Next.Location(i,1);
    XNext(3,i) = (f*b)/d;
    XNext(1,i) = matchedPoints1Next.Location(i,1)*(XNext(3,i)/f);
    XNext(2,i) = matchedPoints1Next.Location(i,2)*(XNext(3,i)/f);
end
% figure, scatter3(XNext(1,:), XNext(2,:), XNext(3,:));

%% RANSAC to find inliers in the feature matching points
reprojErr = 0.2;
numInliers = 0;
inlierMat = zeros(1, length(X));
flag = 0;
for numRansac = 1 : 1500
    inliers = 0;
    inlierMat = zeros(1, length(X));

    indices = randperm(length(Left), 3);
    [M] = estimateRigidTransform(X(:,indices), XNext(:,indices));
    
    for i = 1:length(X) 
        xNew = M*[XNext(:,i); 1];
        xOrg = [X(:,i); 1];
        t = sqrt((xNew(1)-xOrg(1))^2+((xNew(2)-xOrg(2))^2+(xNew(3)-xOrg(3))^2));
        if (t<=reprojErr)
            inliers = inliers + 1;
            inlierMat(i) = i;
        end
    end
    
    
    if(inliers>numInliers)
        numInliers = inliers;
        M2 = M; 
        flag = 1;
    end
    
end

inlierMat= inlierMat(inlierMat~=0);
%figure, showMatchedFeatures(LeftImg, LeftImgNext, matchedPoints1(inlierMat), matchedPoints1Next(inlierMat));

if flag
    incR = M2(1:3,1:3);
    incT = M2(1:3,4);
else
    incR = diag([1,1,1]);
    incT = [0,0,0]';
end

end