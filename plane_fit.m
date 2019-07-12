function [n,R,p] = plane_fit(XYZ)
    % Computes the plane that fits best (lest square of the normal distance to the plane) a set of sample points.
    %
    % INPUTS: 
    % X: a N by 3 matrix where each line is a sample point
    %
    % OUTPUTS:
    % n : a unit (column) vector normal to the plane
    % R : a 3x3 matrix. The 2 first columns of V form an orthonormal basis of the plane and the column is the vector normal to the plane
    % p : a point belonging to the plane
    
    p = mean(XYZ,1);  %the mean of the samples belongs to the plane
    
    %The samples are reduced:
    S = bsxfun(@minus,XYZ,p);
    
    %Computation of the principal directions if the samples cloud
    [V,~] = eig(S'*S);
%     [~,~,V] = svd(S,0);
    
    % Extract the output from the eigenvectors
    n = V(:,1);
    V = V(:,2:end);
    
    % Rotation matrix
    R = [V n]; 
    if n(3) < 0 
        n = -n;
        V(:,1) = -V(:,1);
        R = [V n];
    end
    
end
