function [WS] = moldWorkSpace(P1 ,P2, P3, delta_synthetic, Nx, Ny)
    
    % synthetic points
    P1_synthetic = P1 + delta_synthetic;
    P2_synthetic = P2 + delta_synthetic;
    P3_synthetic = P3 + delta_synthetic;
    
    % vectors
    vec_x = (P3_synthetic - P1_synthetic);
    vec_y = (P2_synthetic - P1_synthetic);

    WS_initial = P1_synthetic + vec_x/(2*Nx) + vec_y/(2*Ny);
    WS = [];
    step_x = 1/(Nx-1);
    step_y = 1/(Ny-1);
    for i=0:step_x:(1-step_x)
       for j=0:step_y:(1-step_y)
          WS = [WS; WS_initial + (vec_x*i + vec_y*j)];
       end      
    end

end