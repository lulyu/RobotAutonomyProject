addpath('./estimateRigidTransform/estimateRigidTransform');

%Load the mat file containing all image names
files = load('./sensor_data/hand_carry.mat');
LeftImgs = files.left_image_names;
RightImgs = files.right_image_names;

%Read the first pair of left and right images
LeftImg = imread('./sensor_data/left000.jpg');
RightImg = imread('./sensor_data/right000.jpg');

%Convert images to grayscale
LeftImg = rgb2gray(LeftImg);
RightImg = rgb2gray(RightImg);

%Detect Harris Corner Features in the images
points1 = detectHarrisFeatures(LeftImg);
points2 = detectHarrisFeatures(RightImg);

%Extrcat the neighbourhood features
[features1, valid_points1] = extractFeatures(LeftImg, points1);
[features2, valid_points2] = extractFeatures(RightImg, points2);

%Match features in the stereo pair
indexPairs = matchFeatures(features1, features2);

%Retrieve locations of points in corresposnign images
matchedPoints1 = valid_points1(indexPairs(:, 1), :);
matchedPoints2 = valid_points2(indexPairs(:, 2), :);

%Visualize the matched features
% showMatchedFeatures(LeftImg, RightImg, matchedPoints1, matchedPoints2);

%Camera Intrinsic Matrix
K = [164.255034407511 0.0 214.523999214172; 0.0 164.255034407511 119.433252334595; 0.0 0.0 1.0];
f = 164.255034407511;

%Stereo Baseline
b = 0.1621;  %%(in m)

%3D point cloud generation
X=zeros(3,matchedPoints1.Count);
for i = 1 :matchedPoints1.Count
    d = matchedPoints1.Location(i,1)-matchedPoints2.Location(i,1);
    X(3,i) = (f*b)/d;
    X(1,i) = matchedPoints1.Location(i,1)*(X(3,i)/f);
    X(2,i) = matchedPoints1.Location(i,2)*(X(3,i)/f);
end

%3D plot of initial point matches
%scatter3(X(1,:),X(2,:),X(3,:))

R = eye(3);
incRprevious = R;
T = zeros(3,1);
M1 = eye(3,4);
p = [0;0;0;1];
P = zeros(4,634);
X = zeros(634,1);
Y = zeros(634,1);
Z = zeros(623, 1);
%Incremental translation and rotation for t and t+1
for i=1:634
    i
    LeftImg     = imread(fullfile('./sensor_data',LeftImgs{i}));
    LeftImgNext = imread(fullfile('./sensor_data',LeftImgs{i+1}));

    RightImg     = imread(fullfile('./sensor_data',RightImgs{i}));
    RightImgNext = imread(fullfile('./sensor_data', RightImgs{i+1}));
    
    [M, incR, incT] = incremental1(LeftImg, LeftImgNext, RightImg, RightImgNext);
    incRprevious = incR;
    
    X(i) = atan2(incR(3,2), incR(3,3));
    Y(i) = atan2(-incR(3,1), sqrt((incR(3,2))^2+(incR(3,3))^2));
    Z(i) = atan2(incR(2,1), incR(1,1));
    p = M*p;
    P(:,i) = p;
    
    R=R*M(1:3,1:3);
    T=T+M(1:3,4);

end

figure, plot3(P(1,:),P(2,:),P(3,:));



