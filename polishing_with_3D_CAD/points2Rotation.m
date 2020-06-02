function [R] = points2Rotation(P1, P2, P3)
    % Compute the vectors of the desired rotation matrix
    nx = (P3 - P1)/norm(P3 - P1);
    ny = (P2 - P1)/norm(P2 - P1);
    nz = cross(nx, ny);
    nz = nz/norm(nz);
       
    % Complete the rotation matrix (orthogonality characteristic)
    ny = cross(nz,nx);
    ny = ny/norm(ny);
            
    % Get the desired rotation
    R = [nx,ny,nz];
end
