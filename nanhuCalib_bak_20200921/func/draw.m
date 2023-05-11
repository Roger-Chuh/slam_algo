function [ept] = draw(img,lines,orders)

I=img;
len=length(orders);

%     figure, imshow(I),hold on 
%     if exist('name','var') 
%          title(name);
%     end
    v=1;
        for k = 1:len
            if orders(k)~=0
%                 xy = [lines(orders(k)).point1; lines(orders(k)).point2];
                xy = [lines(orders(k)).point1; lines(orders(k)).point2];
                
                ept(v,:) = [xy(:,1) ; xy(:,2)];%  第二张图像的[x1,x2,y1,y2]
                
%                 plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','red');
%                 text((xy(1,1)+xy(2,1))/2,(xy(1,2)+xy(2,2))/2,num2str(k));
                v = v+1;
            end   
        end
%     hold off;

end