import gtsam.*;
addpath('lib');
addpath('..\P2B\cmu_16662_p2\geometry_utils\');
load('cmu_16662_p3\NSHLevel2_data.mat');
load('..\ukf_18-Mar-2015 13-31-52.mat');
STARTING_FRAME = 0;
NUM_OF_STEP = 100;
G = 9.8;

%% ----------- Load the sensor data and generating vertex ---------
%load('..\P2B\cmu_16662_p2\sensor_data\hand_carry.mat');
imu_vertex_ind = zeros(1,size(image_timestamps,2));
imu_vertex_ind(1,1) = 1;
for i=2:size(image_timestamps,2)
    % find index corresponding to a time of ts seconds
    ts = image_timestamps(1,i);
    imu_vertex_ind(1,i) = find(imu_timestamps < ts,1,'last');
end

bias_ind = zeros(1,size(imu_timestamps,2));
bias_ind(1,1) = 1;
for i=2:size(imu_timestamps,2)
    % find index corresponding to a time of ts seconds
    ts = imu_timestamps(1,i);
    bias_ind(1,i) = find(ukf_timestamps < ts,1,'last');
end
% Calculate inertial prediction
bias_accel = [0.00,0,0]';
bias_angvel = [0.00,-0.00,0]';
body_accel = rotation_imu_to_leftcam * body_accel;
body_angvel = rotation_imu_to_leftcam * body_angvel;

%% ----------- Known parameters -----------------------
% Defining intrinsic matrix
K = [164.255034407511, 0, 214.523999214172; ...
     0, 164.255034407511, 119.433252334595; ...
     0, 0, 1];
% Using images and matching pairs from the left/right camera to compute, so the R,t for the right camera is known
% Find R and t for the right camera, to compute P_right
R = diag([1,1,1]);
t = [-0.1621,0,0]';
P_left = K*cat(2,R,[0,0,0]');
P_right = K*cat(2,R,t);
R_last = R;

% % ----------- Finding important features -----------------
load('test1-2000d2.mat');
% [feature, R_his, t_his] = findFeature_d2(NUM_OF_STEP, STARTING_FRAME, K, P_left, P_right);
% % %% ----------- Finding landmarks --------------------------
% landmark = {};
% [landmark, landmark_flag_each_frame] = findLandmark(feature, 5, STARTING_FRAME + NUM_OF_STEP);
% load('test.mat');
% load('test50-630_Rt.mat');
% load('test50-630_S.mat');
% load('test50-630_feature.mat');
% load('test400-800d2.mat');



%% ------------ Start to build graph ----------------------
% Create graph container and add factors to it
graph = [];
graph = NonlinearFactorGraph;

%% Create realistic calibration and measurement noise model
% format: fx fy skew cx cy baseline
K = Cal3_S2Stereo(164.255034407511, 164.255034407511, 0, 214.523999214172, 119.433252334595, 0.1621);
stereo_model = noiseModel.Isotropic.Sigma(3,5);

%% Read metadata and compute relative sensor pose transforms
% Initialization
currentPoseGlobal = Pose3(Rot3, Point3); % initial pose is the reference frame (navigation frame)
currentVelocityGlobal = LieVector([0;0;0]); % the vehicle is stationary at the beginning
currentBias = imuBias.ConstantBias(bias_accel, bias_angvel);
sigma_init_x = noiseModel.Isotropic.Precisions([ 0.5; 0.5; 0.5; 1; 1; 1 ]);
sigma_init_v = noiseModel.Isotropic.Sigma(3, 1000.0);
sigma_init_b = noiseModel.Isotropic.Sigmas([ 0.500; 0.500; 0.500; 5.00e-03; 5.00e-03; 5.00e-03 ]);
% ??
sigma_between_b = [ 0.03 * ones(3,1); 0.01 * ones(3,1) ];
w_coriolis = [0;0;0];

% Create initial estimate for camera poses and landmarks
initialEstimate = Values;
R_imu2lcam = diag([1,1,1]);
t_imu2lcam = [0,0,0]';
R_lc2world = diag([1,1,1]);
t_lc2world = [0,0,0]';
R_imu2world = R_imu2lcam*R_lc2world;
t_imu2world = R_imu2lcam*t_imu2lcam + t_lc2world; %?
vo_t = [];
vo_R{STARTING_FRAME+1} = diag([1 1 1]);

imuIndex_of_frame = zeros(1,NUM_OF_STEP);
for measurementIndex = STARTING_FRAME+1:STARTING_FRAME+NUM_OF_STEP-1
    % At each non=IMU measurement we initialize a new node in the graph
    currentPoseKey = symbol('x',measurementIndex);
    t = image_timestamps(1, measurementIndex);
 
  if measurementIndex == STARTING_FRAME+1
    %% Create initial estimate and prior on initial pose, velocity, and biases
    imuIndex_of_frame(measurementIndex-STARTING_FRAME+1) = 1;
    
    priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
    graph.add(NonlinearEqualityPose3(symbol('x',measurementIndex), Pose3()));
    initialEstimate.insert(currentPoseKey, currentPoseGlobal);

    graph.add(PriorFactorPose3(currentPoseKey, currentPoseGlobal, sigma_init_x));
  else
    t_previous = image_timestamps(1, measurementIndex-1);
    %% Summarize IMU data between the previous GPS measurement and now
    IMUindices = find(imu_timestamps >= t_previous & imu_timestamps <= t);
    
    [imu_position_step, imu_velocity_step, imu_orientation_step] = IMUCompute_step(IMUindices, imu_timestamps, body_accel, body_angvel, G, bias_accel, bias_angvel);
    odometry = Pose3(Rot3(ZYXtoE(imu_orientation_step(:,IMUindices(end)+1)')), Point3(imu_position_step(:,IMUindices(end)+1))); % create a measurement for both factors (the same in this case)
    odometryNoise = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.2; 0.1; 0.1; 0.1]); % 20cm std on x,y, 0.1 rad on theta
    graph.add(BetweenFactorPose3(currentPoseKey-1, currentPoseKey, odometry, odometryNoise));
    
    % IMU as position initialization
%     initialEstimate.insert(currentPoseKey, Pose3(Rot3(rotation_imu_to_leftcam * ZYXtoE(imu_orientation(:,measurementIndex)')),  Point3(rotation_imu_to_leftcam * imu_position(:,measurementIndex))));
%     initialEstimate.insert(currentVelKey, LieVector(rotation_imu_to_leftcam * imu_velocity(:,measurementIndex)));
%     initialEstimate.insert(currentBiasKey, currentBias);

    % VO as position initialization
    R_lc2world = R_lc2world*R_his{measurementIndex};
    t_lc2world = t_lc2world+R_lc2world*t_his(measurementIndex,:)';
    vo_R{measurementIndex} = R_lc2world;
    vo_t(:,measurementIndex) = t_lc2world;
%     initialEstimate.insert(currentPoseKey, Pose3(Rot3(ZYXtoE(imu_orientation(:,measurementIndex)')),  Point3(imu_position(:,measurementIndex))));
    
    %if landmark_flag_each_frame(1,measurementIndex)
        initialEstimate.insert(currentPoseKey, Pose3(Rot3(R_lc2world),  Point3(t_lc2world)));
    %end
%initialEstimate.insert(currentPoseKey, Pose3(Rot3(),  Point3()));

  end
   
end % Adding IMU factors

for landmarkIndex = 1:size(landmark,2)
    landmark_i = landmark{landmarkIndex};
    for i=1:landmark_i.num_of_detection
        if landmark_i.detected_images(i)<STARTING_FRAME+NUM_OF_STEP && landmark_i.detected_images(i)>STARTING_FRAME
            graph.add(GenericStereoFactor3D(StereoPoint2(landmark_i.coord_in_l(i,1), landmark_i.coord_in_r(i,1), landmark_i.coord_in_l(i,2)/2+landmark_i.coord_in_r(i,2)/2), ...
                        stereo_model, symbol('x',landmark_i.detected_images(i)), symbol('l',landmark_i.id), K));
        end
    end
    if landmark_i.detected_images(1)<STARTING_FRAME+NUM_OF_STEP && landmark_i.detected_images(i)>STARTING_FRAME
        % Use Triangulation as initial estimation    
        R_lc2world = vo_R{landmark_i.detected_images(1)};
        t_lc2world = vo_t(:,landmark_i.detected_images(1));
        if landmark_i.id==48
            t;
        end
        landmark_i = find3DCoord(landmark_i, R_lc2world, t_lc2world, P_left, P_right);
        coord = landmark_i.coord_3d_world;
        %coord = [landmark_i.coord_3d_world(3) -landmark_i.coord_3d_world(1) -landmark_i.coord_3d_world(2)]';
        initialEstimate.insert(symbol('l',landmark_i.id),Point3(coord));
    end
end

 %% optimize
fprintf(1,'Optimizing\n'); tic
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
toc

%% visualize initial trajectory, final trajectory, and final points
cla; hold on;
axis normal
%axis([-50 50 -50 50 -10 20]);
axis equal
view(-38,12)
camup([0;1;0]);

plot3DTrajectory(initialEstimate, 'r', 1, 0.3);
plot3DTrajectory(result, 'g', 1, 0.3);
%plot3DPoints(result);



