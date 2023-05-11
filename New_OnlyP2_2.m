function angleModal = New_OnlyP2_2(obj,s,existPt1,existPt2,ambiguifyRadius,intrMat,T_B2C)
                
                tmp = zeros(1,s.angleBin);
                for angleIdx = 1 : s.angleBin

                    bodyAngle =  s.angleOrginRange(angleIdx);% body_angle_between_two_imgs + var_angle ;
                    cc = cosd(bodyAngle) ;
                    ss = sind(bodyAngle) ;   
                    % p3D = [z.*p1;ones(1,length(z))];
                    T_B = ...
                    [
                        cc , 0 , ss , 0. ;
                        0 , 1 , 0 , 0.;
                        -ss  , 0 , cc  , 0.;
                        0 0 0 1];
                    T = T_B2C*T_B*inv(T_B2C);


                    matT = T(1:3,4);
                    matSkewT = [   0      -matT(3)   matT(2); ...
                                 matT(3)     0      -matT(1); ...
                                -matT(2)   matT(1)       0;];
                    matR = T(1:3,1:3);
                    matE = matSkewT*matR;
                    matF = inv(intrMat)'*matE*inv(intrMat);
                    L = matF*existPt1.pt;            

                    existPt2toL = abs(sum(existPt2.pt .* L,1)) ./ vecnorm(L(1:2,:));
                    QQidx = find(vecnorm(L(1:2,:)) == 0);
                    if size(QQidx,2)~=0
                        existPt2.pt(QQidx)
                        existPt1.pt(QQidx)
                    end
                    inlierIdx = find(existPt2toL <= ambiguifyRadius);
                    if ~isempty(inlierIdx)
                         intersectPt2toL = 2.*sqrt(ambiguifyRadius.^2 - existPt2toL(inlierIdx).^2);
                         tmp(angleIdx) = tmp(angleIdx)+sum(intersectPt2toL);
                    end
                end
%                 figure;
%                 histogram(tmp);
                obj.angleModal = tmp./sum(tmp);
                angleModal = obj.angleModal;
        end
        