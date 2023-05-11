clear all
close all

SAVE_VID = false;
if SAVE_VID
    vid = VideoWriter('LocalPlanner.avi');
    vid.FrameRate = 30;
    open(vid);
end

obstacle = GetObstacle();

position = [5,0, pi/2];


localPlanner = LocalPlanner();
fovx = pi/6;
maxDepth = 0.2;
dt = 0.1;



figure(1)
plot(obstacle(:,1),obstacle(:,2),'.b'),hold on;
plot(position(1),position(2),'*r'),hold on;
tri = VisionWindow(position,maxDepth,fovx);
plot([tri(1:end,1);tri(1,1)],[tri(1:end,2);tri(1,2)],'-r');hold on;


axis([0 10 0 10])
trace = [];
trace(end+1,:) = position;

for i = 1:5000
[v,w] = ObstacleAvoidance(localPlanner, obstacle, position);
if v == 0 && w~=0
    position(3) = position(3) + w*dt;
end
if v~=0 && w == 0
    position(1:2) = position(1:2) + v*dt*[cos(position(3)),sin(position(3))];
end

trace(end+1,:) = position;
figure(1),clf
plot(obstacle(:,1),obstacle(:,2),'.b'),hold on;
% plot(position(1),position(2),'*r'),hold on;
plot(trace(:,1),trace(:,2),'-g'),hold on;
tri = VisionWindow(position,maxDepth,fovx);
plot(position(1),position(2),'or'),hold on;
plot([tri(1:end,1);tri(1,1)],[tri(1:end,2);tri(1,2)],'-r');hold on;
% plot(localPlanner.nextRoadMap(1),localPlanner.nextRoadMap(2),'or');hold on
axis([0 10 0 10])

drawnow;

if SAVE_VID
    img = getframe(figure(1));
    ResizedFrame = img.cdata;
    ResizedFrame = imresize(img.cdata,[480, 640],'nearest');
    writeVideo(vid, ResizedFrame);
end
end
if SAVE_VID
    close(vid)
end

