aviobj=VideoWriter(fullfile(pwd,'demo.avi')); 
aviobj.FrameRate = 15;
open(aviobj);
done = false;
figNum = 10;
while ~done
    
    
     figure(figNum);
     saveas(gcf,'temp.png');
     frameImg = imread('temp.png');
    
    writeVideo(aviobj,imresize(frameImg, [1275, 1116]));
end
close(aviobj);