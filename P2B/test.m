load('data3.mat');
camera_pos = [];
camera_pos(1,:) = [0,0,0];
R_last = diag([1 1 1]);
figure
xlabel('x');ylabel('y');zlabel('z');
axis equal;
hold on
for pts_num=2:size(R_his,2)
    t = t_his(pts_num,:);
    thx = thx_his(pts_num,:);
    thy = thy_his(pts_num,:);
    thz = thz_his(pts_num,:);
    R = R_his{pts_num};
    t_rec = [t(1) t(2) t(3)];
    camera_pos(pts_num,:) = bsxfun(@plus, camera_pos(pts_num-1,:), (R_last*t_rec')');
    plot3(camera_pos(pts_num,1),camera_pos(pts_num,2),camera_pos(pts_num,3),'go');
    R_last = R_last*R;
    t_length(pts_num,:) = norm(t);
    pause(0.002);
    pts_num
end




figure
plot(t_length);
figure
hold on
plot(thx_his,'b');
plot(thy_his,'g');
plot(thz_his,'r');

