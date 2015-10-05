function [landmark_id, landmark_feature] = findLandmarks(main_features, main_feature_relations, ...
                                                        main_feature_2Dcor_table)
num_of_landmarks = 0;
num_of_entries = 0;
for i=1:size(main_features,1)
    ind_relation = find(main_feature_relations(:,2) == i);
    if size(ind_relation,1)>10
        num_of_landmarks = num_of_landmarks + 1;
        landmark_id(num_of_landmarks) = i;
        landmark_features(num_of_landmarks) = main_features(i,:);
        for j=1:size(ind_relation,1)
            num_of_entries = num_of_entries + 1;
            
            landmark_2Dcor_table(num_of_entries,:) = [pic_num-1,num_of_main_features];
        end

    end
end