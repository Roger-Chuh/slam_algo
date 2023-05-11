function [angZ, angX, tvLen] = TransVec2Pol(transVec)

tvLen = norm(transVec);
v = transVec / tvLen;
angZ = acos(v(3));
if (v(3) == 0)
    angX = 0;
else
    angX = acos(v(1)/sqrt(1-v(3)^2));
    if (v(2)<0)
        angX = -angX;
    end
end

end
