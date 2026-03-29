function DCM_if = lg2dcm_if(LG)

DCM_if = [
    cos(LG) sin(LG) 0;
    -sin(LG) cos(LG) 0;
    0 0 1;
    ];
end
