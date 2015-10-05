function [] = plotGraph(landmark)
figure;
hold on;
for i=1:size(landmark,2)
    l = landmark{i};
    for j=1:l.num_of_detection
        plot(l.id, 1, 'go');
        plot(l.detected_images(j), 5, 'ro')
        plot([l.id l.detected_images(j)], [1 5]);
    end
end