function [imu_position_all, imu_velocity_all, imu_orientation_all] = IMUCompute(imu_timestamps, body_accel, body_angvel, G, bias_accel, bias_angvel)

imu_position_all=zeros(3,size(imu_timestamps,2));
imu_velocity_all=zeros(3,size(imu_timestamps,2));
imu_orientation_all=zeros(3,size(imu_timestamps,2));

for imu_t=1:size(imu_timestamps,2)-1
        delta_t = imu_timestamps(imu_t+1)-imu_timestamps(imu_t);
        R_nb = ZYXToR(imu_orientation_all(:,imu_t));
        f_n = R_nb'*(body_accel(:,imu_t) - bias_accel); %- bias_accel(:,imu_t)); % - bias(f)
        imu_velocity_all(:,imu_t+1) = imu_velocity_all(:,imu_t)+f_n*delta_t+[0,G,0]'*delta_t;
        imu_position_all(:,imu_t+1) = imu_position_all(:,imu_t)+imu_velocity_all(:,imu_t)*delta_t;
        E_nb = ZYXtoE(imu_orientation_all(:,imu_t));
        imu_orientation_all(:,imu_t+1) = imu_orientation_all(:,imu_t)+E_nb'*(body_angvel(:,imu_t) - bias_angvel)*delta_t;
%         imu_orientation_all(:,imu_t+1) = imu_orientation_all(:,imu_t)+E_nb'*(body_angvel(:,imu_t) - bias_angvel(:,imu_t))*delta_t;
end