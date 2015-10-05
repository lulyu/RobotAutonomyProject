X1 = []; X2 = [];
X1 = cat(1,X_matched,ones(1,size(X_matched,2)));
X2 = cat(1,X_next_matched,ones(1,size(X_next_matched,2)));
figure
plot3(X1(1,:),X1(2,:),X1(3,:),'go');
hold on
plot3(X2(1,:),X2(2,:),X2(3,:),'ro');
for i=1:size(X1,2)
    line([X1(1,i),X2(1,i)],[X1(2,i),X2(2,i)],[X1(3,i),X2(3,i)],'color','b'); % Defined Z
end
xlabel('x');ylabel('y');zlabel('z');
axis equal;