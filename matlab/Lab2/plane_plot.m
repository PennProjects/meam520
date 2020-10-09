
%method 1
% Vertex coordinates
p1=[0 0 0];
p2=[0 0 10];
p3=[30.3860 47.293 161.1350];
% % Plot trianle using 'patch'
% figure('color','w')
% h=patch('Faces',1:3,'Vertices',[p1;p2;p3]);
% set(h,'FaceColor','r','EdgeColor','k','LineWidth',2,'FaceAlpha',0.5)
% % axis equal vis3d
% view([30 30])
% hold on
% plot3(p1(1), p1(2), p1(3), '.')
% xlabel('x','FontSize',20)
% ylabel('y','FontSize',20)
% zlabel('z','FontSize',20)


%method 2
pointA = p1;
pointB = p2;
pointC = p3;
normal = cross(pointA-pointB, pointA-pointC); %# Calculate plane normal
%# Transform points to x,y,z
x = [pointA(1) pointB(1) pointC(1)];  
y = [pointA(2) pointB(2) pointC(2)];
z = [pointA(3) pointB(3) pointC(3)];

%Find all coefficients of plane equation    
A = normal(1); B = normal(2); C = normal(3);
D = -dot(normal,pointA);
%Decide on a suitable showing range
xLim = [min(x) max(x)];
zLim = [min(z) max(z)];
[X,Z] = meshgrid(xLim,zLim);
Y = (A * X + C * Z + D)/ (-B);
reOrder = [1 2  4 3];
figure();patch(X(reOrder),Y(reOrder),Z(reOrder),'b');
grid on;
view([30 30]);
alpha(0.3);

hold on
plot3(0, 10, 20,'o')

xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');