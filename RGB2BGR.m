function RGB = RGB2BGR(BGR)
    temp(:,:,1) = BGR(:,:,3);
    temp(:,:,2) = BGR(:,:,2);
    temp(:,:,3) = BGR(:,:,1);
    RGB = temp;
end