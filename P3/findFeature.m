function [feature, R_his, t_his] = findFeature(NUM_OF_STEP, STARTING_FRAME, K, P_left, P_right)
%% --------------- Starting point ----------------------
filepath_left = 'D:\Study\Robot Autonomy\Project\P2B\cmu_16662_p2\sensor_data\left';
filepath_right = 'D:\Study\Robot Autonomy\Project\P2B\cmu_16662_p2\sensor_data\right';
camera_pos = zeros(NUM_OF_STEP,3);
t_his = zeros(NUM_OF_STEP,3);
thx_his = zeros(NUM_OF_STEP,1);
thy_his = zeros(NUM_OF_STEP,1);
thz_his = zeros(NUM_OF_STEP,1);
R_his = {};
R = diag([1,1,1]);
R_last = R;

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
[X_next,~] = triangulate(P_left,pts_next_left.Location(indexPairs_next_lr(:,1),:)',P_right,pts_next_right.Location(indexPairs_next_lr(:,2),:)');   
% Starting point coordinate
camera_pos(1,:) = [0,0,0];
t_his(1,:) = 0;
%pts_num = 1;

disp('Starting extracting graph data...');
%% --------- Initialization of all main_feature tables -----------
num_of_main_features = 0;
% num_of_entries = 0;
main_features = [];
feature = {};
% main_feature_relations = [];
% main_feature_2Dcor_table = [];
                
for pic_num=(STARTING_FRAME+1):STARTING_FRAME+NUM_OF_STEP-1
    %% ---------- Initialization ----------------
    pic_num = pic_num+1;
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
%     pts_next_left = detectHarrisFeatures(img_next_left_gray);
%     pts_next_right = detectHarrisFeatures(img_next_right_gray);
    pts_next_left = detectSURFFeatures(img_next_left_gray);
    pts_next_right = detectSURFFeatures(img_next_right_gray);
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
    % showMatchedFeatures(img_left,img_right,pts_left(indexPairs_lr(:,1),:),pts_right(indexPairs_lr(:,2),:));
    % figure;
    % showMatchedFeatures(img_left,img_next_left,pts_left(indexPairs_ll(:,1),:),pts_next_left(indexPairs_ll(:,2),:));
    % PlotMatches( img_left, pts_left.Location(indexPairs_lr(:,1),:), img_right, pts_right.Location(indexPairs_lr(:,2),:))
    
    X = X_next;
    [X_next,~] = triangulate(P_left,pts_next_left.Location(indexPairs_next_lr(:,1),:)',P_right,pts_next_right.Location(indexPairs_next_lr(:,2),:)');   
    num_pts_reproj = 0;
    %% Finding corresponsing 3D points in the two time steps to find features
    for i=1:size(indexPairs_ll,1)
        ind1 = indexPairs_ll(i,1); ind2 = indexPairs_ll(i,2);
        pair1 = find(eq(indexPairs_lr(:,1),ind1));
        pair2 = find(eq(indexPairs_next_lr(:,1),ind2));
        if (~isempty(pair1) && ~isempty(pair2))
            feature_id = [];
            % Try to find a recurring main_feature
            if ~isempty(main_features)
                [feature_id,~] = matchFeatures(features_left(pair1,:),single(main_features),'MatchThreshold',2,'MaxRatio',0.6);
            end
            if isempty(feature_id) % new main_feature
                num_of_main_features = num_of_main_features + 1;
                feature{num_of_main_features} = FeaturePoint(num_of_main_features, pic_num-1, pts_left.Location(indexPairs_lr(pair1,1),:), pts_right.Location(indexPairs_lr(pair1,2),:), features_left(pair1,:));
                % showMatchedFeatures(img_left,img_right,pts_left(indexPairs_lr(pair1,1),:),pts_right(indexPairs_lr(pair1,2),:));
                main_features(num_of_main_features,:) = features_left(pair1,:);
            else
                feature{feature_id(2)} = feature{feature_id(2)}.addImageDetected(pic_num-1, pts_left.Location(indexPairs_lr(pair1,1),:), pts_right.Location(indexPairs_lr(pair1,2),:));
            end
        end
        if (~isempty(pair1))
            num_pts_reproj = num_pts_reproj +1;
            X_matched_2(:,num_pts_reproj) = X(:,pair1);
            pts_matched_1(num_pts_reproj,:) = pts_left.Location(ind1,:);   
            pts_matched_2(num_pts_reproj,:) = pts_next_left.Location(ind2,:);   
        end
    end
    
%     if size(pts_matched_2,1)>=12
%         R_last = R*R_last;
%         [thx, thy, thz] = RtoEulerAngles(R_last);
%         [R, t, inliers] = ransac_reproject(pts_matched_2, X_matched_2, K);
%     else
%         R_last = R*R_last;
%         t = [0,0,0]';
%     end
%     [~, R, t] = incremental1(img_left, img_next_left, img_right, img_next_right);
    
    %% Record trajactory
% 	camera_pos(pic_num,:) = bsxfun(@plus, camera_pos(pic_num-1,:), (inv(R_last)*t)');
    
%     t_his(pic_num,:) = t;
%     thx_his(pic_num,:) = thx;
%     thy_his(pic_num,:) = thy;
%     thz_his(pic_num,:) = thz;
%     R_his{pic_num} = R;
t_his = [];
R_his = [];

    
    %% Output status
    disp(strcat('Now step... ',num2str(pic_num),'...'));
    disp(strcat('    Number of Harris Corners detected - l:',num2str(size(pts_next_left,1)),' r:',num2str(size(pts_next_right,1)),';'));
    disp(strcat('    Number of 3D points found in next frame:',num2str(size(indexPairs_next_lr,1)),';'));
    disp(strcat('    Number of matches found between steps:',num2str(size(indexPairs_ll,1)),';'));
    %disp(strcat('    Number of 3D matches found between steps:',num2str(size(X_matched,2)),';'));
    %disp(strcat('    Norm t:',num2str(norm(t)),';'))
end