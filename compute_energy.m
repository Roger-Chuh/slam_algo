classdef compute_energy < handle
    properties
        
        
    end
    methods (Static)
        function compute_energy_data(frame_index, sequence, window_side, sigma_c, sigma_d)
            window_side = 10;
            sigma_c = 1;
            sigma_d = 2.5;
            
            t = frame_index;
            start = 0;
            endd = sequence.end - sequence.start;
            if t - window_side < start
                window_start = start;
                window_end = min(endd, window_start + window_side*2 + 1);
            elseif t + window_side + 1> endd
                window_end = endd;
                window_start = max(start, window_end - window_side*2 - 1);
            else
                window_start = t - window_side;
                window_end = t + window_side + 1;
            end
            
            h = sequence.height;
            w = sequence.width;
            levels = sequence.depth_levels;
            depth_values = linspace(sequence.depth_min, sequence.depth_max, sequence.depth_levels);
            I_t = sequence.I(t);
            x_h = homogeneous_coord_grid(h,w);
            d = zeros(h, w);
            L = zeros(levels, h, w);
            
            for t_prime = window_start : window_end
                % skip if same image
                if t_prime == t: continue
                end
    
        % get image at frame i
        I_t_prime = sequence.I(t_prime);
            
            
             for level = 1 :  length(levels)
            % photo-consistency constraint ------------------------------------
            
            % fill each position with the current level depht value
            d[:,:] = depth_values[level] 

            % compute conjugate pixel position using the depth level at time t
            x_prime_h = conujugate_coordinates(
                    sequence=sequence,
                    pose1=t,
                    pose2=t_prime,
                    coorsxy=x_h, 
                    d=d)
            
            % remap the image w.r.t. the projected pixels
            x_prime = np.transpose(x_prime_h[:2,:,:], [1,2,0])
            I_t_prime_projected = cv2.remap(
                    src=I_t_prime, 
                    map1=x_prime, 
                    map2=None, 
                    interpolation=cv2.INTER_NEAREST, 
                    borderValue=[128, 128, 128])
            
            % compute the photo constraint term (pc)
            color_difference = L2_norm(I_t, I_t_prime_projected, keepdims=False)
            pc = sigma_c/(sigma_c + color_difference)
            
            % check if bundle optimization is required
            if not sequence.use_bundle():
                % update likelyhood using photo-consistency contraint
                L[level,:,:] += pc

            else:
                % geometric-consistency constraint ----------------------------
                
                % compute candidate pixels using previously estimated depth values
                depth_indices = sequence.D[t_prime] # get prev. estimated depth 
                depth_indices_projected = cv2.remap(
                        src=depth_indices, 
                        map1=x_prime, 
                        map2=None, 
                        interpolation=cv2.INTER_NEAREST, 
                        borderValue=int(levels/2.0))
                
                % fill the matrix d with the conjugate pixels' depth value indices
                np.take(depth_values, depth_indices_projected, out=d)
                
                % project back from t_prime to t using prev. estimated depth values
                projected_x_prime_h = conujugate_coordinates(
                        sequence=sequence,
                        pose1=t_prime,
                        pose2=t,
                        coorsxy=x_prime_h,
                        d=d)
    
                % compute norm of diff. between original and projected coord 
                color_difference_norm = np.sum(
                    np.square(x_h - projected_x_prime_h), 
                    axis=0,
                    keepdims=False)
                
                % compute the bundle optimization term (pv)
                pv = np.exp(color_difference_norm/(-2*sigma_d*sigma_d))
                
                % update likelyhood using photo and geometric constraints
                L[level,:,:] += pc*pv
          
        end
        
    end
    methods
        function XYZ = homogeneous_coord_grid(h, w)
            
            [X, Y] = meshgrid(1:w,1:h);
            Z = ones(size(X));
            
            XYZ = permute(cat(3, X,Y,Z),[3 1 2]);
            
        end
        
        
    end