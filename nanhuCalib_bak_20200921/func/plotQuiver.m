function plotQuiver(rvecccc,varargin)
if (nargin == 1)
    color1 = [1 0 0];
elseif (nargin == 2)
    color1 = varargin{1};
else
    error('Too many input arguments');
end
quiver3(zeros(size(rvecccc,1),1),zeros(size(rvecccc,1),1),zeros(size(rvecccc,1),1),rvecccc(:,1),rvecccc(:,2),rvecccc(:,3),'Color',color1);axis equal
hold on;plot3(rvecccc(:,1),rvecccc(:,2),rvecccc(:,3),'o', 'Color', color1);
% % quiver3(0,0,0,rvecccc(1,1),rvecccc(1,2),rvecccc(1,3),'Color',[0 0 1],'LineWidth',5);axis equal
% % plot3(rvecccc(1,1),rvecccc(1,2),rvecccc(1,3),'*b','MarkerSize',15,'LineWidth',5);



end