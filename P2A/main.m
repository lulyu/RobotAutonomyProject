addpath('lib');
filepath1 = 'D:\Study\Robot Autonomy\Project\P2A\cmu_16662_camera_calibration\rawleft';
filepath2 = 'D:\Study\Robot Autonomy\Project\P2A\cmu_16662_camera_calibration\rawright';
pic_num = 19;
pic_num = num2str(pic_num,'%04i');

% Define images to process
imageFileNames1 = strcat(filepath1,pic_num,'.jpg');
imageFileNames2 = strcat(filepath2,pic_num,'.jpg');
img1 = imread(imageFileNames1);
img2 = imread(imageFileNames2);

% Finding matching points
% [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints({imageFileNames1, imageFileNames2});
% cpselect(img1,img2);
load('cp19_2.mat');

% Loading intrinsic matrix
load('cameraParams_else.mat');

K1 = cameraParams_left.IntrinsicMatrix';
K2 = cameraParams_right.IntrinsicMatrix';

% Using manually selected points
img1pts = cat(2,img1pts,ones(size(img1pts,1),1));
img2pts = cat(2,img2pts,ones(size(img2pts,1),1));

% Auto select checkerboard points
% img1pts = cat(2,imagePoints(:,:,1),ones(size(imagePoints,1),1));
% img2pts = cat(2,imagePoints(:,:,2),ones(size(imagePoints,1),1));

% pts1 = K1\img1pts';
% pts2 = K2\img2pts';

normalization_constant = max(max([img1pts,img2pts]));
F = eightpoint_norm(img1pts', img2pts', normalization_constant);
E = K1'*F*K2;
[R, t] = EtoRt(E);


displayEpipolarF(img1,img2, F);

