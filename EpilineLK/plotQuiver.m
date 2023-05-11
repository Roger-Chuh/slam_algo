function plotQuiver(data, color, lineWidth,ind)

quiver3(zeros(size(data,1),1), zeros(size(data,1),1), zeros(size(data,1),1), data(:,1), data(:,2), data(:,3),'Color', color, 'LineWidth', lineWidth);
hold on;

for i = 1 : size(data,1)
    plot3(data(i,1), data(i,2), data(i,3), 'o', 'Color', color, 'LineWidth', lineWidth);
%     text(data(i,1), data(i,2), data(i,3), num2str(i));
end
quiver3(0, 0, 0, data(ind,1), data(ind,2), data(ind,3),'Color', [0 1 0], 'LineWidth', 5);

plot3(data(ind,1),data(ind,2),data(ind,3),'og', 'LineWidth', 5)
axis equal;

end