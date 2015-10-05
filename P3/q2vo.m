import gtsam.*;
addpath('lib');
addpath('..\P2B\cmu_16662_p2\geometry_utils\');
load('cmu_16662_p3\NSHLevel2_data.mat');
load('..\ukf_18-Mar-2015 13-31-52.mat');
STARTING_FRAME = 1;
NUM_OF_STEP = 630;
G = 9.8;

%% ----------- Load the sensor data and generating vertex ---------
load('..\P2B\cmu_16662_p2\sensor_data\hand_carry.mat');
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
% [imu_position_all, imu_velocity_all, imu_orientation_all] = IMUCompute(imu_timestamps, body_accel, body_angvel, G, bias_accel,bias_angvel);
% [imu_position_all, imu_velocity_all, imu_orientation_all] = IMUCompute(imu_timestamps, body_accel, body_angvel, G, ukf_accelbias(:,bias_ind), ukf_gyrobias(:,bias_ind));

gt_ind = zeros(1,size(image_timestamps,2));
gt_ind(1,1) = 1;
for i=2:size(image_timestamps,2)
    % find index corresponding to a time of ts seconds
    ts = image_timestamps(1,i);
    gt_ind(1,i) = find(gt_timestamps < ts,1,'last');
end

% imu_position=imu_position_all(:,imu_vertex_ind);
% imu_velocity=imu_velocity_all(:,imu_vertex_ind);
% imu_orientation=imu_orientation_all(:,imu_vertex_ind);

% imu_position=gt_position(:,gt_ind);
% imu_velocity=gt_velocity(:,gt_ind);
% imu_orientation=gt_rpy(:,gt_ind);

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

%% ----------- Finding important features -----------------
% [feature, R_his, t_his] = findFeature(NUM_OF_STEP, STARTING_FRAME, K, P_left, P_right);
% %% ----------- Finding landmarks --------------------------
% landmark = {};
% [landmark] = findLandmark(feature, 5);
% load('test.mat');
% load('test50-630_Rt.mat');
load('test50-630_S.mat');



%% ------------ Start to build graph ----------------------
% Create graph container and add factors to it
graph = [];
graph = NonlinearFactorGraph;

%% Create realistic calibration and measurement noise model
% format: fx fy skew cx cy baseline
K = Cal3_S2Stereo(164.255034407511, 164.255034407511, 0, 214.523999214172, 119.433252334595, 1);
stereo_model = noiseModel.Isotropic.Sigma(3,1);

%% Read metadata and compute relative sensor pose transforms
% IMU metadata
disp('-- Reading sensor metadata')
IMU_metadata = importdata(findExampleDataFile('KittiEquivBiasedImu_metadata.txt'));
IMU_metadata = cell2struct(num2cell(IMU_metadata.data), IMU_metadata.colheaders, 2);
IMUinBody = Pose3.Expmap([IMU_metadata.BodyPtx; IMU_metadata.BodyPty; IMU_metadata.BodyPtz;
  IMU_metadata.BodyPrx; IMU_metadata.BodyPry; IMU_metadata.BodyPrz; ]);
if ~IMUinBody.equals(Pose3, 1e-5)
  error 'Currently only support IMUinBody is identity, i.e. IMU and body frame are the same';
end

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

% Solver object
% isamParams = ISAM2Params;
% isamParams.setFactorization('CHOLESKY');
% isamParams.setRelinearizeSkip(10);
% isam = gtsam.ISAM2(isamParams);
% newValues = Values;

% Create initial estimate for camera poses and landmarks
initialEstimate = Values;
R_imu2lcam = diag([1,1,1]);
t_imu2lcam = [0,0,0]';
R_lc2world = diag([1,1,1]);
t_lc2world = [0,0,0]';
R_imu2world = R_imu2lcam*R_lc2world;
t_imu2world = R_imu2lcam*t_imu2lcam + t_lc2world; %?
vo_t = [];

[imu_position_all, imu_velocity_all, imu_orientation_all] = IMUCompute(imu_timestamps, body_accel, body_angvel, G, bias_accel, bias_angvel);
imuIndex_of_frame = zeros(1,NUM_OF_STEP);
for measurementIndex = STARTING_FRAME:STARTING_FRAME+NUM_OF_STEP-1
    % At each non=IMU measurement we initialize a new node in the graph
    currentPoseKey = symbol('x',measurementIndex);
    currentVelKey =  symbol('v',measurementIndex);
    currentBiasKey = symbol('b',measurementIndex);
    t = image_timestamps(1, measurementIndex);
 
  if measurementIndex == STARTING_FRAME
    %% Create initial estimate and prior on initial pose, velocity, and biases
%     newValues.insert(currentPoseKey, currentPoseGlobal);
%     newValues.insert(currentVelKey, currentVelocityGlobal);
%     newValues.insert(currentBiasKey, currentBias);
    imuIndex_of_frame(measurementIndex-STARTING_FRAME+1) = 1;
    
    graph.add(NonlinearEqualityPose3(symbol('x',measurementIndex), Pose3()));
    initialEstimate.insert(currentPoseKey, currentPoseGlobal);
%     initialEstimate.insert(currentVelKey, currentVelocityGlobal);
%     initialEstimate.insert(currentBiasKey, currentBias);
    
%     graph.add(PriorFactorPose3(currentPoseKey, currentPoseGlobal, sigma_init_x));
%     graph.add(PriorFactorLieVector(currentVelKey, currentVelocityGlobal, sigma_init_v));
%     graph.add(PriorFactorConstantBias(currentBiasKey, currentBias, sigma_init_b));
  else
    t_previous = image_timestamps(1, measurementIndex-1);
    %% Summarize IMU data between the previous GPS measurement and now
    IMUindices = find(imu_timestamps >= t_previous & imu_timestamps <= t);
    imuIndex_of_frame(measurementIndex-STARTING_FRAME+1) = IMUindices(size(IMUindices,2));
    
    currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
      currentBias, IMU_metadata.AccelerometerSigma.^2 * eye(3), ...
      IMU_metadata.GyroscopeSigma.^2 * eye(3), IMU_metadata.IntegrationSigma.^2 * eye(3));
    
%     for imuIndex = IMUindices
%       accMeas = [ body_accel(1,imuIndex); body_accel(2,imuIndex); body_accel(3,imuIndex) ];
%       omegaMeas = [ body_angvel(1,imuIndex); body_angvel(2,imuIndex); body_angvel(3,imuIndex) ];
%       %% ----- EVERYTHING HAS BEEN TRANSFORMED TO LEFT CAMERA FRAME!!! -------------
%       accMeas = rotation_imu_to_leftcam * accMeas;
%       omegaMeas = rotation_imu_to_leftcam * omegaMeas;
%       deltaT = imu_timestamps(imuIndex+1)-imu_timestamps(imuIndex);
%       currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);
%     end
    
    % Create IMU factor
%     graph.add(ImuFactor( ...
%       currentPoseKey-1, currentVelKey-1, ...
%       currentPoseKey, currentVelKey, ...
%       currentBiasKey, currentSummarizedMeasurement, G, w_coriolis));
    
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
    initialEstimate.insert(currentPoseKey, Pose3(Rot3(R_lc2world),  Point3(t_lc2world)));
%     initialEstimate.insert(currentVelKey, LieVector(rotation_imu_to_leftcam * imu_velocity(:,measurementIndex)));
%     initialEstimate.insert(currentBiasKey, currentBias);
    
    % Bias evolution as given in the IMU metadata
%     graph.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), ...
%       noiseModel.Diagonal.Sigmas(sqrt(numel(IMUindices)) * sigma_between_b)));
    
    % Update solver
    % =======================================================================
    % We accumulate 2*GPSskip GPS measurements before updating the solver at
    % first so that the heading becomes observable.
%       isam.update(graph, newValues);
%       graph = NonlinearFactorGraph;
%       newValues = Values;
%       
%       if rem(measurementIndex,10)==0 % plot every 10 time steps
%         cla;
%         plot3DTrajectory(isam.calculateEstimate, 'g-');
%         title('Estimated trajectory using ISAM2 (IMU+GPS)')
%         xlabel('[m]')
%         ylabel('[m]')
%         zlabel('[m]')
%         axis equal
%         drawnow;
%       end
%       % =======================================================================
%       currentPoseGlobal = isam.calculateEstimate(currentPoseKey);
%       currentVelocityGlobal = isam.calculateEstimate(currentVelKey);
%       currentBias = isam.calculateEstimate(currentBiasKey);
  end
   
end % Adding IMU factors

for landmarkIndex = 1:size(landmark,2)
    landmark_i = landmark{landmarkIndex};
    for i=1:landmark_i.num_of_detection
        graph.add(GenericStereoFactor3D(StereoPoint2(landmark_i.coord_in_l(i,1), landmark_i.coord_in_r(i,1), landmark_i.coord_in_l(i,2)/2+landmark_i.coord_in_r(i,2)/2), ...
                    stereo_model, symbol('x',landmark_i.detected_images(i)), symbol('l',landmark_i.id), K));
    end
    % Use Triangulation as initial estimation    
    R_lc2world = vo_R{landmark_i.detected_images(1)};
    t_lc2world = vo_t(:,landmark_i.detected_images(1));
    landmark_i = find3DCoord(landmark_i, R_lc2world, t_lc2world, P_left, P_right);
    coord = landmark_i.coord_3d_world;
    %coord = [landmark_i.coord_3d_world(3) -landmark_i.coord_3d_world(1) -landmark_i.coord_3d_world(2)]';
    initialEstimate.insert(symbol('l',landmark_i.id),Point3(coord));
end

 %% optimize
fprintf(1,'Optimizing\n'); tic
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
toc

%% visualize initial trajectory, final trajectory, and final points
cla; hold on;
axis normal
axis([-50 50 -50 50 -10 20]);
axis equal
view(-38,12)
camup([0;1;0]);

plot3DTrajectory(initialEstimate, 'r', 1, 0.3);
plot3DTrajectory(result, 'g', 1, 0.3);
plot3DPoints(result);



