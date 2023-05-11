function a_k = transformToParameters(T)
R = T(1:3,1:3);
t = T(1:3,4);
theta = acos(0.5*(trace(R)-1));

if (theta > 1e-10)
    a_k(4:6) = theta*(1/(2*sin(theta)))*[R(3,2)-R(2,3), R(1,3)-R(3,1), R(2,1)-R(1,2)];
    w = a_k(4:6);
    w_cross = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
    V_inv = eye(3) - 0.5*w_cross + ((1 - theta*sin(theta)/(2*(1-cos(theta))))/theta^2)*w'*w;
    a_k(1:3) = (V_inv*t)';
else
    a_k(4:6) = [0 0 0];
    a_k(1:3) = t;
end

end