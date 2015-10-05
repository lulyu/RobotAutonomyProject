addpath('lib');
addpath('cmu_16662_p2\geometry_utils\');
FIRST_TIME = 1;
MATCH_OR_TRACK = 2; % 1 -- MATCH; 2 -- TRACK
STARTING_FRAME = 0;
NUM_OF_STEP = 200;

%% ----------- Known parameters -----------------------
% Defining intrinsic matrix
K = [164.255034407511, 0, 214.523999214172; ...
     0, 164.255034407511, 119.433252334595; ...
     0, 0, 1];
 
% Using images and matching pairs from the left/right camera to compute, so the R,t for the right camera is known
% Find R and t for the right camera, to compute P_right
R = diag([1,1,1]);
t = [0.1621,0,0]';
P_left = K*cat(2,R,[0,0,0]');
P_right = K*cat(2,R,t);
R_last = R;

%% --------------- Starting point ----------------------
filepath_left = 'D:\Study\Robot Autonomy\Project\P2B\cmu_16662_p2\sensor_data\left';
filepath_right = 'D:\Study\Robot Autonomy\Project\P2B\cmu_16662_p2\sensor_data\right';
camera_pos = zeros(NUM_OF_STEP,3);
t_his = zeros(NUM_OF_STEP,3);
thx_his = zeros(NUM_OF_STEP,1);
thy_his = zeros(NUM_OF_STEP,1);
thz_his = zeros(NUM_OF_STEP,1);
R_his = {};

pic_num = STARTING_FRAME;
pic_num = num2str(pic_num,'%03i');
% Read the images
imageFileName_next_left = strcat(filepath_left,pic_num,'.jpg');
imageFileName_next_right = strcat(filepath_right,pic_num,'.jpg');
img_next_left = imread(imageFileName_next_left);
img_next_right = imread(imageFileName_next_right);
% Find matching points
img_next_left_gray = rgb2gray(img_next_left);
img_next_right_gray = rgb2gray(img_next_right);
% pts_next_left = detectHarrisFeatures(img_next_left_gray);
% pts_next_right = detectHarrisFeatures(img_next_right_gray);
pts_next_left = detectSURFFeatures(img_next_left_gray);
pts_next_right = detectSURFFeatures(img_next_right_gray);
[features_next_left,validPoints_next_left] = extractFeatures(img_next_left_gray,pts_next_left,'Method','SURF');
[features_next_right,validPoints_next_right] = extractFeatures(img_next_right_gray,pts_next_right,'Method','SURF');
[indexPairs_next_lr,matchmetric_next_lr] = matchFeatures(features_next_left,features_next_right,'MatchThreshold',1,'MaxRatio',0.6);
% Finding 3D coordinates for features
X_next = [];
% [~,~, ind_inliers] = rejectOutliers(pts_next_left.Location(indexPairs_next_lr(:,1),:)', pts_next_right.Location(indexPairs_next_lr(:,2),:)');
% indexPairs_next_lr = indexPairs_next_lr(ind_inliers,:);
[X_next,~] = triangulate(P_left,pts_next_left.Location(indexPairs_next_lr(:,1),:)',P_right,pts_next_right.Location(indexPairs_next_lr(:,2),:)');   
%X_next = bsxfun(@times, X_next, sign(X_next(1,:)));
% [X_next,err_next] = triangulate_batch(pts_next_left.Location(indexPairs_next_lr(:,1),:),pts_next_right.Location(indexPairs_next_lr(:,2),:),P_left,P_right); 

% Starting point coordinate
camera_pos(1,:) = [0,0,0];
t_his(1,:) = 0;
pts_num = 1;

disp('Starting trajactory calculation...');
for pic_num=(STARTING_FRAME+1):NUM_OF_STEP
    %% ---------- Initialization ----------------
    pts_num = pts_num+1;
    pic_num_next = pic_num;
    pic_num_next = num2str(pic_num_next,'%03i');

    % Define images to process
    imageFileNames_left = imageFileName_next_left;
    imageFileNames_right = imageFileName_next_right;
    imageFileName_next_left = strcat(filepath_left,pic_num_next,'.jpg');
    imageFileName_next_right = strcat(filepath_right,pic_num_next,'.jpg');

    img_left = img_next_left;
    img_right = img_next_right;
    img_next_left = imread(imageFileName_next_left);
    img_next_right = imread(imageFileName_next_right);
    

    %% Finding Matching Points
    img_left_gray = img_next_left_gray;
    img_right_gray = img_next_right_gray;
    img_next_left_gray = rgb2gray(img_next_left);
    img_next_right_gray = rgb2gray(img_next_right);

    % Finding matching points
    pts_left = pts_next_left;
    pts_right = pts_next_right;
    pts_next_left = detectHarrisFeatures(img_next_left_gray);
    pts_next_right = detectHarrisFeatures(img_next_right_gray);
%     pts_next_left = detectSURFFeatures(img_next_left_gray);
%     pts_next_right = detectSURFFeatures(img_next_right_gray);
    features_left = features_next_left; % Only need to keep the left feature for calculate the incremental step for the left cam
    [features_next_left,validPoints_next_left] = extractFeatures(img_next_left_gray,pts_next_left,'Method','SURF');
    [features_next_right,validPoints_next_right] = extractFeatures(img_next_right_gray,pts_next_right,'Method','SURF');
    
    indexPairs_lr = indexPairs_next_lr;
    matchmetric_lr = matchmetric_next_lr;
    [indexPairs_next_lr,matchmetric_next_lr] = matchFeatures(features_next_left,features_next_right,'MatchThreshold',1,'MaxRatio',0.6);
    % indexPairs_ll is ESSENTIAL to find matching feature between two time
    % steps, later we need to find features that both exist in indexPairs_lr and
    % indexPairs_next_lr, then use them to find the scaled transform.
    [indexPairs_ll,matchmetric_ll] = matchFeatures(features_left,features_next_left,'MatchThreshold',1,'MaxRatio',0.6);

    %% Visualize the matched 2D points
    % showMatchedFeatures(img_next_left,img_next_right,pts_next_left(indexPairs_next_lr(:,1),:),pts_next_right(indexPairs_next_lr(:,2),:));
    % figure;
    % showMatchedFeatures(img_left,img_next_left,pts_left(indexPairs_ll(:,1),:),pts_next_left(indexPairs_ll(:,2),:));
    % PlotMatches( img_left, pts_left.Location(indexPairs_lr(:,1),:), img_right, pts_right.Location(indexPairs_lr(:,2),:))

    %% Finding The 3D coordinates of the features in next step - Triangulation
    X = X_next; 
%     [~,~, ind_inliers] = rejectOutliers(pts_next_left.Location(indexPairs_next_lr(:,1),:)', pts_next_right.Location(indexPairs_next_lr(:,2),:)');
%     indexPairs_next_lr = indexPairs_next_lr(ind_inliers,:);
    [X_next,~] = triangulate(P_left,pts_next_left.Location(indexPairs_next_lr(:,1),:)',P_right,pts_next_right.Location(indexPairs_next_lr(:,2),:)');   
    %X_next = bsxfun(@times, X_next, sign(X_next(1,:)));
    %[X_next,err_next] = triangulate_batch(pts_next_left.Location(indexPairs_next_lr(:,1),:),pts_next_right.Location(indexPairs_next_lr(:,2),:),P_left,P_right);
    
%     %% Visualize 3D points
%     figure
%     plot3(X(3,:),X(1,:),-X(2,:),'go');
%     axis equal;
%     axis([0,5,-5,5,-5,5]);
%     line([0,2],[0,0],'color','g'); % Defined Z
%     line([0,0],[0,2],'color','b'); % Defined X
%     line([0,0],[0,0],[0,2],'color','r'); % Defined Y
    
    %% Find R and t between the two steps
    % Tracking, trying to find R and t between two time steps
%     F = estimateFundamentalMatrix(pts_left(indexPairs_ll(:,1),:),pts_next_left(indexPairs_ll(:,2),:),'Method','RANSAC','DistanceThreshold',0.001);
%     E = K'*F*K;
%     [R, t_up2scale] = EtoRt(E);
    
    %% Finding corresponsing 3D points in the two time steps to determine 't'
    k=1;kk=1;
    X_matched = []; X_next_matched = []; pts_matched = []; pts_matched1 = [];
    for i=1:size(indexPairs_ll,1)
        ind1 = indexPairs_ll(i,1); ind2 = indexPairs_ll(i,2);
        pair1 = find(eq(indexPairs_lr(:,1),ind1));
        pair2 = find(eq(indexPairs_next_lr(:,1),ind2));
        if (~isempty(pair1) && ~isempty(pair2))
            X_matched(:,k) = X(:,pair1);
            X_next_matched(:,k) = X_next(:,pair2);
            pts_matched1(k,:) = pts_left.Location(ind1,:);
            pts_matched(k,:) = pts_next_left.Location(ind2,:);
            k=k+1;
        end
        if (~isempty(pair1))
            X_matched_2(:,kk) = X(:,pair1);
            pts_matched_2(kk,:) = pts_next_left.Location(ind2,:);
            kk=kk+1;
        end
    end
%     X1 = []; X2 = [];
%     X1 = cat(1,X_matched,ones(1,size(X_matched,2)));
%     X2 = cat(1,X_next_matched,ones(1,size(X_next_matched,2)));
%     [X_matched, X_next_matched, ind_accepted] = rejectOutliers(X_matched, X_next_matched);
%     [R, t, ~,~] = ransac_3drotate(X_matched, X_next_matched);
%     [thx, thy, thz] = RtoEulerAngles(R_last);
    
    if size(pts_matched,1)>=12
        R_last = R*R_last;
        [thx, thy, thz] = RtoEulerAngles(R_last);
        [R, t, inliers] = ransac_reproject(pts_matched_2, X_matched_2, K);
    else
        R_last = R*R_last;
        t = [0,0,0]';
    end

%     scale = norm(t_3d)/norm(t);
    
    %% Record trajactory
	camera_pos(pts_num,:) = bsxfun(@plus, camera_pos(pts_num-1,:), (inv(R_last)*t)');
    
    t_his(pts_num,:) = t;
    thx_his(pts_num,:) = thx;
    thy_his(pts_num,:) = thy;
    thz_his(pts_num,:) = thz;
    R_his{pts_num} = R;
    
    %% Output status
    disp(strcat('Now step... ',num2str(pts_num),'...'));
    disp(strcat('    Number of Harris Corners detected - l:',num2str(size(pts_next_left,1)),' r:',num2str(size(pts_next_right,1)),';'));
    disp(strcat('    Number of 3D points found in next frame:',num2str(size(indexPairs_next_lr,1)),';'));
    disp(strcat('    Number of matches found between steps:',num2str(size(indexPairs_ll,1)),';'));
    disp(strcat('    Number of 3D matches found between steps:',num2str(size(X_matched,2)),';'))
    disp(strcat('    Norm t:',num2str(norm(t)),';'))
end


figure
plot3(camera_pos(:,1),camera_pos(:,2),camera_pos(:,3),'go');
xlabel('x');ylabel('y');zlabel('z');
axis equal;


