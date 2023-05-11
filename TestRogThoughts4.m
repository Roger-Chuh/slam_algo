function TestRogThoughts4()

intrMat = [500 0 320;0 500 240;0 0 1];

thetaList0 = [1:1:90];
thetaList = [zeros(1, 100) thetaList0 thetaList0(end)*ones(1,100)];

id1 = find(thetaList == 0);
id2 = find(thetaList == thetaList(end));
id2 = id2(2:end);

idRotate = id1(end)+1 : id2(1)-1;

path1 = [zeros(length(id1),1) zeros(length(id1),1) 10*id1'];


radius = 500;
path2 = + [radius*cosd(thetaList0)' zeros(length(thetaList0),1) radius*sind(thetaList0)'];
path2 = path2 - path2(1,:);
path2 = path2 + path1(end,:);

path3 = [[path2(end,1) : -10: (path2(end,1) -10*(length(id2)-1))]'  zeros(length(id2),1)  path2(end,3)*ones(length(id2),1)];
% figure,plot(path2(:,1), path2(:,3));axis equal;hold on;plot(path1(:,1),path1(:,3));plot(path3(:,1),path3(:,3))

Path = [path1; path2;path3];

b2c = rodrigues([0.1;0.2;0.3]);rotx(-1); 

errStack = [];
% B2C_stack

for kkk = 1 : 5000
    
    pt3dCam = 1000*[rand(1000,2)-0.5 10*rand(1000,1)];
    
    b2c = rodrigues(0.003*(rand(3,1)-0.5));rotx(-1);
    b2c = rodrigues(3*(rand(3,1)-0.5));rotx(-1);
    B2C_stack(:,kkk) = rodrigues(b2c);
    
    pt3dBody = (inv(b2c)*pt3dCam')';
    % load('D:\Temp\20200831\rMatNew3.mat')
    % b2c = rMatNew(1:3,1:3);
    % tStraight = [0;0;100];
    
    camMat = [reshape(eye(3), 1,[]) 0 0 0];
    cnt = 1;
    for i = 1 : size(Path,1)
        
        bodyMat(i,:) = [reshape(roty(-thetaList(i)),1, 9) Path(i,:)];
        Tbody = [b2c*roty(-thetaList(i)) Path(i,:)'; 0 0 0 1];
        %     camMat(i,:) = [reshape(Tbody(1:3,1:3),1, 9) Tbody(1:3,4)'];
        
        if i > 1
            TCur_body = [reshape(bodyMat(i,1:9),3,3) bodyMat(i,10:12)';0 0 0 1];
            TPrv_body = [reshape(bodyMat(i-1,1:9),3,3) bodyMat(i-1,10:12)';0 0 0 1];
            deltaBody(:,:,cnt) = inv(TCur_body) * TPrv_body;
            deltaCam(:,:,cnt) = [b2c [0;0;0];0 0 0 1] * deltaBody(:,:,cnt) * [b2c' [0;0;0];0 0 0 1];
            
            ang = rad2deg(norm(rodrigues(deltaCam(1:3,1:3,cnt))));
            %         deltaBody(:,:,cnt) = roty(-ang);
            
            lastMat = [reshape(camMat(end,1:9),3,3) camMat(end,10:12)';0 0 0 1];
            newMat_inv = lastMat*inv(deltaCam(:,:,cnt));
            camMat = [camMat; reshape(newMat_inv(1:3,1:3), 1, 9) newMat_inv(1:3, 4)'];
            
            cnt = cnt + 1;
        end
        
    end
    
    B = fitplane(camMat(:,10:12)');
    B = B./norm(B(1:3));
    B = sign(B(2)).*B;
    err = dot(repmat(B,1,size(camMat,1)), pextend(camMat(:,10:12)'));
    
    b2cNew = CorrectRot([0;1;0], B(1:3))*roty(69);
    
    
    
    rotRange = [102:189];
%     rotRange = [1:189];
    camMatRot_c2w = camMat(rotRange,:);
    bodyMatRot_b2w = [];
    for jjj = 1 : size(camMatRot_c2w, 1)
        
        T_c2w = [reshape(camMatRot_c2w(jjj,1:9),3,3) camMatRot_c2w(jjj,10:12)';0 0 0 1];
        T_w2c = inv(T_c2w);
        ptProj = TransformAndProject(pt3dCam,intrMat, T_w2c(1:3,1:3), T_w2c(1:3,4));
        pt2d(:,jjj,1) = ptProj(:,1);
        pt2d(:,jjj,2) = ptProj(:,2);
        
        rotAng = rad2deg(norm(rodrigues(T_w2c(1:3,1:3))));
        
        if norm(rodrigues(roty(rotAng)*T_w2c(1:3,1:3)')) > norm(rodrigues(roty(-rotAng)*T_w2c(1:3,1:3)'))
            rotAng = -rotAng;
        end
        
%         T_w2c_comp = [(b2c) [0 0 0]'; 0 0 0 1]*[roty(rotAng) T_w2c(1:3,4); 0 0 0 1]*[inv(b2c) [0 0 0]'; 0 0 0 1];
        T_w2c_comp = [b2c*roty(10)*roty(rotAng)*(b2c*roty(10))' T_w2c(1:3,4); 0 0 0 1];
        
        compErr(jjj,:) = norm(T_w2c_comp-(T_w2c));
        T_w2b = [inv(b2c) [0 0 0]'; 0 0 0 1]*T_w2c*[b2c [0 0 0]'; 0 0 0 1];
        T_b2w = inv(T_w2b);
        bodyMatRot_b2w = [bodyMatRot_b2w; [reshape(T_b2w(1:3,1:3),1,9) T_b2w(1:3,4)']];
    end
%     figure,subplot(1,2,1);plotPath(camMatRot_c2w);title('cam path');subplot(1,2,2);plotPath(bodyMatRot_b2w);title('body path');
    
    
    if kkk == 1
        
        b2cVec = [0 0 0]';
        
        Err0 = CostFunc(deltaCam, deltaBody, b2cVec);
        
        options1= optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',10000,'TolX',1e-18, 'OptimalityTolerance', 1e-17,'FunctionTolerance', 1e-16,'StepTolerance',1e-15,'MaxFunEvals',10000, 'MaxIterations',5000000);
        [b2cVecOpt,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) CostFunc(deltaCam, deltaBody, X),[b2cVec],[],[],options1);
        
        b2cOpt = rodrigues(b2cVecOpt);
        b2cOpt(:,2) - b2c(:,2)
        
        %     b2cNew = CorrectRot([0;1;0], B(1:3));
        
        Err1 = CostFunc(deltaCam, deltaBody, b2cVecOpt);
        
        Err2 = CostFunc(deltaCam, deltaBody, rodrigues(b2c));
        
        Err3 = CostFunc(deltaCam, deltaBody, rodrigues(b2cNew));
        
    end
    a = b2cNew'*b2c;
    
    bbb = acos(a(1,1));
    err_comp1 = norm(([rot_y1(bbb) - a]));
    err_comp2 = norm(([rot_y2(bbb) - a]));
    if 0
        err_comp3 = [rot_y3(bbb) - a];
        err_comp4 = [rot_y4(bbb) - a];
    else
        err_comp11 = norm(([rot_y1(bbb) - a']));
        err_comp22 = norm(([rot_y2(bbb) - a']));
    end
    
    
    err_comp3 = norm(([rot_y3(bbb) - a]));
    err_comp33 = norm(([rot_y3(bbb) - a']));
    
    errStack = [errStack [err_comp1;err_comp11;err_comp2; err_comp22; err_comp3; err_comp33; rad2deg(norm(rodrigues(b2c)))]];
end

figure,subplot(1,2,1);plot(min(errStack));subplot(1,2,2),plot(errStack(end,:));
return
a1 = a;
a2 = a1; a2(2,2) = -a1(2,2);
temp_a1 = rot_y1(rad2deg(norm(rodrigues(a1))));
temp_a2 = rot_y2(rad2deg(norm(rodrigues(a2))));

aa1 = b2cNew*temp_a1;
aa2 = b2cNew*temp_a2;
err_b2c1 = aa1 - b2c;
err_b2c2 = aa2 - b2c;


figure,plot([Err0 Err1])

figure,plotPath(camMat);
figure,plot(camMat(:,11))
figure,plotPath(bodyMat)


end
function Err = CostFunc(deltaCam, deltaBody, b2cVec)
Err = zeros(size(deltaCam, 3), 1);

b2c = rodrigues(b2cVec);
for i = 1 : size(deltaCam, 3)
    tempCam = deltaCam(1:3,1:3,i);
    tempBody = deltaBody(1:3,1:3,i);
    tempCam_comp = b2c(1:3,1:3) * tempBody * b2c(1:3,1:3)';
    Err(i,:) = rad2deg(norm(rodrigues(tempCam_comp' * tempCam)));
end
end
function R = rot_y1(theta)

R = [cos(theta) 0 sin(theta); 0 1 0;-sin(theta) 0 cos(theta)];

end
function R = rot_y2(theta)

R = [cos(theta) 0 -sin(theta); 0 -1 0;-sin(theta) 0 -cos(theta)];

end
function R = rot_y3(theta)

R = [cos(theta) 0 sin(theta); 0 -1 0;sin(theta) 0 -cos(theta)];

end

function R = rot_y4(theta)

R = [cos(theta) 0 sin(theta); 0 -1 0;-sin(theta) 0 cos(theta)];

end