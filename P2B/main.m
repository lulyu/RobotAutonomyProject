FIRST_TIME = 1;
MATCH_OR_TRACK = 1; % 1 -- MATCH; 2 -- TRACK
pic_num = 97;

%% Initialization
addpath('lib');

if (MATCH_OR_TRACK==1)
    pic_num_next = pic_num;
    filepath1 = 'D:\Study\Robot Autonomy\Project\P2B\cmu_16662_p2\sensor_data\left';
    filepath2 = 'D:\Study\Robot Autonomy\Project\P2B\cmu_16662_p2\sensor_data\right';
    pic_num = num2str(pic_num,'%03i');
    pic_num_next = num2str(pic_num_next,'%03i');
else
    pic_num_next = pic_num+1;
    filepath1 = 'D:\Study\Robot Autonomy\Project\P2B\cmu_16662_p2\sensor_data\left';
    filepath2 = 'D:\Study\Robot Autonomy\Project\P2B\cmu_16662_p2\sensor_data\left';
    pic_num = num2str(pic_num,'%03i');
    pic_num_next = num2str(pic_num_next,'%03i');
end

% Define images to process
imageFileNames1 = strcat(filepath1,pic_num,'.jpg');
imageFileNames2 = strcat(filepath2,pic_num_next,'.jpg');
img1 = imread(imageFileNames1);
img2 = imread(imageFileNames2);

%% Finding Matching Points
if FIRST_TIME
    img1_gray = rgb2gray(img1);
    img2_gray = rgb2gray(img2);

    % Finding matching points
    pts1 = detectHarrisFeatures(img1_gray);
    pts2 = detectHarrisFeatures(img2_gray);
    [features1,validPoints1] = extractFeatures(img1_gray,pts1,'Method','SURF');
    [features2,validPoints2] = extractFeatures(img2_gray,pts2,'Method','SURF');
    [indexPairs,matchmetric] = matchFeatures(features1,features2,'MatchThreshold',1);
else
    load('matchingPairs.mat');
end

showMatchedFeatures(img1,img2,pts1(indexPairs(:,1),:),pts2(indexPairs(:,2),:));
% PlotMatches( img1, pts1.Location(indexPairs(:,1),:), img2, pts2.Location(indexPairs(:,2),:))

%% Finding The Camera Matrixs 

% Defining intrinsic matrix
K = [164.255034407511, 0, 214.523999214172; ...
     0, 164.255034407511, 119.433252334595; ...
     0, 0, 1];
 
if (MATCH_OR_TRACK==1) % Matching pair, so the R,t for the right camera is known
    % Find R and t for the right camera, to compute P2
    R = diag([1,1,1]);
    t = [0.1621,0,0]';
    P1 = K*cat(2,R,[0,0,0]');
    P2 = K*cat(2,R,t);
else % Tracking, trying to find R and t between two time steps
    F = estimateFundamentalMatrix(pts1(indexPairs(:,1),:),pts2(indexPairs(:,2),:),'Method','RANSAC');
    [tform,inlierpoints1,inlierpoints2] = ...
        estimateGeometricTransform(pts1(indexPairs(:,1),:),pts2(indexPairs(:,2),:),'projective');
end

%% Triangulation
[X,err] = triangulate_batch(pts1.Location(indexPairs(:,1),:),pts2.Location(indexPairs(:,2),:),P1,P2);
figure
plot3(X(3,:),X(1,:),-X(2,:),'go');
axis([0,4,-4,4,-4,4]);
line([0,2],[0,0],'color','g'); % Defined Z
line([0,0],[0,2],'color','b'); % Defined X
line([0,0],[0,0],[0,2],'color','r'); % Defined Y

% pts1 = K1\img1pts';
% pts2 = K2\img2pts';




