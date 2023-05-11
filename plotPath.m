function plotPath(data)
hold on;
scale = 0.5;
if 0
    
    % %     pcshow(UU(:,UU(1,:)<XX & UU(3,:)<ZZ)', [1 0 0], 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    % %     legend('direction','proposed method','sfm reference');
    % %     xlabel('X (mm)');
    % %     ylabel('Y (mm)');
    % %     zlabel('Z (mm)');
    % %     set(gca,  'CameraUpVector',[0 -1 0],'DataAspectRatio',[1 1 1]);
    % %     title('Reconstructed 3-D Scene');
    quiver3(data(:,10), data(:,11), data(:,12), scale.*data(:,1), scale.*data(:,2), scale.*data(:,3),'Color', [1 0 0]);
    quiver3(data(:,10), data(:,11), data(:,12), scale.*data(:,4), scale.*data(:,5), scale.*data(:,6),'Color', [0 1 0]);
    quiver3(data(:,10), data(:,11), data(:,12), scale.*data(:,7), scale.*data(:,8), scale.*data(:,9),'Color', [0 0 1]);
    legend('x','y','z');
    xlabel('x(mm)');ylabel('y(mm)');zlabel('z(mm)')
    % hold on;
    % for i = 1 : size(data,1)
    %     plot3(data(i,1), data(i,2), data(i,3), 'o', 'Color', color, 'LineWidth', lineWidth);
    % %     text(data(i,1), data(i,2), data(i,3), num2str(i));
    % end
    axis equal;
else
    pcshow([data(1,10), data(1,11), data(1,12)], [0 0 1], 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    % %    scatter3(XYZtemp(:,1),XYZtemp(:,2),XYZtemp(:,3),'filled'); axis equal;
    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');
    set(gca,  'CameraUpVector',[0 -1 0],'DataAspectRatio',[1 1 1]);
    title('Reconstructed 3-D Scene');
    hold on
    
    %     quiver3(poseVec11(1:kl,1),poseVec11(1:kl,2),poseVec11(1:kl,3),poseVec11(1:kl,4),poseVec11(1:kl,5),poseVec11(1:kl,6),0.1);
    %     quiver3(poseVec11(1:kl,1),poseVec11(1:kl,2),poseVec11(1:kl,3),poseVec11(1:kl,7),poseVec11(1:kl,8),poseVec11(1:kl,9),0.1);
    %     quiver3(poseVec11(1:kl,1),poseVec11(1:kl,2),poseVec11(1:kl,3),poseVec11(1:kl,10),poseVec11(1:kl,11),poseVec11(1:kl,12),0.1);
    
    
    if 1
        quiver3(data(:,10), data(:,11), data(:,12), scale.*data(:,1), scale.*data(:,2), scale.*data(:,3),scale);
        quiver3(data(:,10), data(:,11), data(:,12), scale.*data(:,4), scale.*data(:,5), scale.*data(:,6),scale);
    end
    quiver3(data(:,10), data(:,11), data(:,12), scale.*data(:,7), scale.*data(:,8), scale.*data(:,9),scale);
    
    
    
    % %    quiver3(poseVec11Robot(1:kl,1),poseVec11Robot(1:kl,2),poseVec11Robot(1:kl,3),poseVec11Robot(1:kl,4),poseVec11Robot(1:kl,5),poseVec11Robot(1:kl,6),0.1);
    % %    quiver3(poseVec11Robot(1:kl,1),poseVec11Robot(1:kl,2),poseVec11Robot(1:kl,3),poseVec11Robot(1:kl,7),poseVec11Robot(1:kl,8),poseVec11Robot(1:kl,9),0.1);
    % %    quiver3(poseVec11Robot(1:kl,1),poseVec11Robot(1:kl,2),poseVec11Robot(1:kl,3),poseVec11Robot(1:kl,10),poseVec11Robot(1:kl,11),poseVec11Robot(1:kl,12),0.1);
    
    legend('world origin','pnp x','pnp y', 'pnp z');
    if 0
        plot3(data(:,10), data(:,11), data(:,12),'.','MarkerSize',15,'LineWidth',15)
    end
    if 1
        plot3(data(1,10), data(1,11), data(1,12),'sb','MarkerSize',15,'LineWidth',15);
    end
end

end