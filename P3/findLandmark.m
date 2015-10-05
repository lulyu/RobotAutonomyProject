function [landmark, landmark_flag_each_frame] = findLandmark(feature, num_threshold, num_of_frames)
landmark = {};
landmark_flag_each_frame = zeros(1,num_of_frames);
num_of_landmarks = 0;
for i=1:size(feature,2)
    if feature{i}.num_of_detection>num_threshold && abs(feature{i}.coord_in_l(1,2)-feature{i}.coord_in_r(1,2))<1
        num_of_landmarks = num_of_landmarks + 1;
        landmark{num_of_landmarks} = feature{i};
        for j=1:feature{i}.num_of_detection
            landmark_flag_each_frame(1,feature{i}.detected_images(j)) = 1;
        end
    end
end