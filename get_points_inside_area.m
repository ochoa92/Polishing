function points = get_points_inside_area(area, p)    
    % Computes the points inside an area delimited by 4 points (P1,P2,P3,P4)
    %
    % INPUTS:
    % area: a 4x3 matrix with 4 points (P1,P2,P3,P4) that delimit the work-space;
    % p: a 3xN matrix with points that need to be restricted to the defined area.
    %
    % OUTPUTS:
    % points: a 3xN matrix with points inside the area.

    % points
    P1 = area(1,1:2);
    P2 = area(2,1:2);
    P3 = area(3,1:2);
    P4 = area(4,1:2);
    
    % points vectors
    l12 = P2 - P1;
    l23 = P3 - P2;
    l34 = P4 - P3;
    l41 = P1 - P4;
    
    % vectors betwen points and p
    l1P = [];
    l2P = [];
    l3P = [];
    l4P = [];
    num_p = length(p);
    points = [];
    for i=1:num_p
        p_ = p(i,1:2);
        l1P = p_ - P1;
        l2P = p_ - P2;
        l3P = p_ - P3;
        l4P = p_ - P4;
        
        % signal between the 2 vectors
       signal1 = dot(l1P,l12);
       signal2 = dot(l2P,l23);
       signal3 = dot(l3P,l34);
       signal4 = dot(l4P,l41);

       if signal1 >= 0.0 && signal2 >= 0.0 && signal3 >= 0.0 && signal4 >= 0.0
           points = [points; p(i,:)];
       end
    end
    
end
        