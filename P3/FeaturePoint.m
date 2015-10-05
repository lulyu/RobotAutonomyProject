classdef FeaturePoint
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id = 0;
        num_of_detection = 0;
        detected_images = []; % A list of number denoting in which frames is this feature detected.
        coord_in_l = [];
        coord_in_r = [];
        feature = [];
        coord_3d_world =[];
    end
    
    methods
        function obj = FeaturePoint(id, image_num, coord_in_l, coord_in_r, feature)
            obj.id = id;
            obj.detected_images(1) = image_num;
            obj.coord_in_l(1,:) = coord_in_l;
            obj.coord_in_r(1,:) = coord_in_r;
            obj.feature = feature;
            obj.num_of_detection = 1;
        end
    end
    methods
        function obj1 = addImageDetected(obj, image_num, coord_in_l, coord_in_r)
            obj1 = obj;
            obj1.num_of_detection = obj.num_of_detection + 1;
            obj1.detected_images(obj1.num_of_detection) = image_num;
            obj1.coord_in_l(obj1.num_of_detection,:) = coord_in_l;
            obj1.coord_in_r(obj1.num_of_detection,:) = coord_in_r;
        end
        
        function obj1 = find3DCoord(obj, R_lc2world, t_lc2world, P_left, P_right)
            obj1 = obj;
            [X,~] = triangulate(P_left, obj.coord_in_l(1,:)', P_right, obj.coord_in_r(1,:)');   
            X_world = R_lc2world*X + t_lc2world;
            obj1.coord_3d_world = X_world;
        end
    end
    
end

