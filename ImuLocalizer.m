classdef ImuLocalizer < handle
    
    properties
        %         prevTimeStamp;
        %         inputDecoder;
        %         robotPosePlotHdl;
        % %         prvImgTimeStamp;
        % %         curImgTimeStamp;
        t;
        gravity;
        accSkewMat;
        gyroSkewMat;
        accScaleMat;
        gyroScaleMat;
        accBias;
        gyroBias;
        imuPara;
        RotIntg;  % integrated rotation matrix
        VIntg;  % integrated velocity
        PIntg;  % integrated position
        transVecI2W;
        rotMatI2W;
        gScale;
        motionState;
        id;
    end
    
    methods
        function obj = ImuLocalizer(gravity,deltT,RotIntg0)
            
            %             obj.prvImgTimeStamp = 0;
            %             obj.curImgTimeStamp = 0;
            obj.accSkewMat = eye(3);
            obj.gyroSkewMat = eye(3);
            obj.accScaleMat = eye(3);
            obj.gyroScaleMat = eye(3);
            obj.accBias = zeros(3,1);
            obj.gyroBias = zeros(3,1);
            obj.t = deltT;
            obj.imuPara.accCov_ = diag([0.01 0.01 0.01]);
            obj.imuPara.gyroCov_ = diag([0.0012 0.0012 0.0012]);
            obj.imuPara.bAccCov_ = diag([1 1 1]);
            obj.imuPara.bGyroCov_ = diag([1 1 1]);
            
            obj.gravity = gravity;
            obj.RotIntg = RotIntg0;
            obj.VIntg = [0 0 0]';
            obj.PIntg = [0 0 0]';
            obj.transVecI2W = [0 0 0];
            obj.rotMatI2W = reshape(eye(3),1,9);
            obj.gScale = 1; %9.81;
            obj.motionState = 0; % [1 2 3] [left straight right]
            obj.id = 1;
        end
        
        function PreintegrateOneFrame( obj,  imuAll,  prvImgTimeStamp,  curImgTimeStamp)
            %             prvImgTimeStamp = obj.prvImgTimeStamp;
            %             curImgTimeStamp = obj.curImgTimeStamp;
            if 0 % 7725
                idd = find(imuAll(:,1) >= prvImgTimeStamp & imuAll(:,1) < curImgTimeStamp);
                w = deg2rad([-imuAll(idd,3) imuAll(idd,2) imuAll(idd,4)]);
                a = [-(imuAll(idd,6)) (imuAll(idd,5)) (imuAll(idd,7))];
            else % buildEnv
                idd = find(imuAll(:,1) >= prvImgTimeStamp & imuAll(:,1) < curImgTimeStamp);
                w = ([imuAll(idd,2) imuAll(idd,3) imuAll(idd,4)]);
                a = [(imuAll(idd,5)) (imuAll(idd,6)) (imuAll(idd,7))];
            end
            obj.id = idd;
            
            accCorrect = obj.gScale.*((obj.accSkewMat*obj.accScaleMat*a' - repmat(obj.accBias,1,length(idd)))');
            gyroCorrect = (obj.gyroSkewMat*obj.gyroScaleMat*w' - repmat(obj.gyroBias,1,length(idd)))';
            PIM = ImuLocalizer.ResetPreintegrate(obj.RotIntg, obj.VIntg, obj.transVecI2W(end,:)' );
            R0 = obj.RotIntg;
            for i = 1 : size(accCorrect,1)
                PIM = ImuLocalizer.Preintegrate(PIM,accCorrect(i,:)',gyroCorrect(i,:)',obj.imuPara, obj.t);
            end
            
            dt = size(accCorrect,1)*obj.t;
            obj.RotIntg = PIM.R_;
            obj.PIntg = PIM.p_;
            
            obj.VIntg = - obj.gScale*obj.gravity' * dt + eye(3) * PIM.v_;
            
            obj.transVecI2W = [obj.transVecI2W; ( - 0.5 * obj.gScale*obj.gravity' * (dt)^2 + eye(3) * obj.PIntg)'] ;
            obj.rotMatI2W = [obj.rotMatI2W; reshape(eye(3) * PIM.R_,1,9)];
            
            R1 = obj.RotIntg;
            rotVec = rodrigues(R1'*R0);
            if rotVec(2) < 0 && abs(rotVec(2)) > 0.02
                obj.motionState = 3; % turning right
            end
            if rotVec(2) > 0 && abs(rotVec(2)) > 0.02
                obj.motionState = 1;  % turning left
            end
            if abs(rotVec(2)) < 0.01
                obj.motionState = 2;  % forward or still
            end
            
        end
        
        
    end
    methods (Static)
        function o = ResetPreintegrate(rotmat, velo, posi)
            o.R_ = rotmat;
            o.p_ = posi;
            %             o.v_ = inv(o.R_)*velo;
            o.v_ = eye(3)*velo;
            o.phiv_ = SO3.log(o.R_);  %zeros(3,1);
            o.ba_ = zeros(3,1);  %o.phiv_;
            o.bg_ = zeros(3,1); %o.phiv_;
            o.cov_ = zeros(9);
            o.t_ = 0;
            o.DR_bg_ = zeros(3);
            o.Dv_bg_ = o.DR_bg_;
            o.Dp_bg_ = o.DR_bg_;
            o.Dv_ba_ = o.DR_bg_;
            o.Dp_ba_ = o.DR_bg_;
        end
        function o = Preintegrate(o,acc,gyro,imuPara,t)
            o.t_ = o.t_ + t;
            I3 = eye(3);
            z3 = zeros(3);
            t22 = t*t/2;
            %correct gyro measurement with bias
            gyroCorrect = gyro - o.bg_;
            %calculate the orientation change between consecutive time slots
            if 0
                Rk_k_1 = SO3.exp(gyroCorrect*t);
            else
                Rk_k_10 = SO3.exp(gyroCorrect*t);
                Rk_k_1 = rodrigues(gyroCorrect*t);
                [Rk_k_10 - Rk_k_1];
                skjnkv = 1;
            end
            Rk_k_1T = Rk_k_1';
            %calculate Jacobian of Jr
            Jrk_k_1 = SO3.Dexp(gyroCorrect*t);
            %correct acc measurement with bias
            accCorrect = acc - o.ba_;
            accCorrectX = SO3.skew(accCorrect);
            
            %update state covariance with new IMU measurements
            Ak = [Rk_k_1T,z3,z3;
                -o.R_*accCorrectX*t22,I3,I3*t;
                -o.R_*accCorrectX*t,z3,I3];
            Bk = [z3;
                o.R_*t22;
                o.R_*t];
            Ck = [Jrk_k_1;
                z3;
                z3];
            o.cov_ = Ak*o.cov_*Ak' + Bk*imuPara.accCov_*Bk' + Ck*imuPara.gyroCov_*Ck';
            
            DR_bg = o.DR_bg_;
            Dv_bg = o.Dv_bg_;
            Dp_bg = o.Dp_bg_;
            Dv_ba = o.Dv_ba_;
            Dp_ba = o.Dp_ba_;
            
            o.DR_bg_ = Rk_k_1T*DR_bg - Jrk_k_1*t;
            %             o.Dv_bg_ = Dv_bg - o.R_*accCorrectX*DR_bg*t22;
            o.Dv_bg_ = Dv_bg - o.R_*accCorrectX*DR_bg*t;
            o.Dv_ba_ = Dv_ba - o.R_*t;
            
            
            o.Dp_bg_ = Dp_bg + Dv_bg*t - o.R_*accCorrectX*DR_bg*t22;
            o.Dp_ba_ = Dp_ba + Dv_ba*t - o.R_*t22;
            
            %update navigation states with IMU measurements
            R = o.R_;
            o.R_ = o.R_*Rk_k_1;
            o.phiv_ = SO3.log(o.R_);
            o.p_ = o.p_ + o.v_*t + R*accCorrect*t22;
            o.v_ = o.v_ + R*accCorrect*t;
        end
    end
    methods
        % Implementation of abstract methods in Configurable base class
        function SetDefaultValue(obj, cfgParam)
            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
    
end