function [R] = normal2Rotation(normal) 
    % Computes a Rotation matrix from a normal of a plane
    %
    % INPUTS:
    % normal: 1x3 vector
    %
    % OUTPUTS:
    % R: a rotation matrix 3x3    


    nz = normal;
    nz = nz/norm(nz);
    
    nx = [-1 0 0];
    nx = nx/norm(nx);
       
    ny = cross(nz,nx);
    ny = ny/norm(ny);
    
    nx = cross(ny,nz);
    nx = nx/norm(nx);
        
    R = [nx' ny' nz'];

end