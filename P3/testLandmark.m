ifhavelm = zeros(1,630);
for i=1:size(landmark,2)
    for j=1:size(landmark{i}.num_of_detection)
        ifhavelm(landmark{i}.detected_images(j))=1;
    end
end
find(ifhavelm)