function [p_inside, p_inside_index] = get_npoints_inside_area(area, p)    
    % Computes the points inside an area delimited by 'n' points
    %
    % INPUTS:
    % area: a nx3 matrix with 'n' points that delimit the area;
    % p: a 3xN matrix with points that need to be restricted to the defined area.
    %
    % OUTPUTS:
    % p_inside: a 3xN matrix with points inside the area;
    % p_inside_index: N matrix with index of the points inside the area.

    % area points
    num_area = length(area);
    area_ = area(:,1:2);
        
    % vectors that connect the area points
    vec_area = [];
    for i=1:(num_area-1)
        vec_area = [vec_area; area_(i+1,:) - area_(i,:)];
    end
    
    % vectors betwen area points and each point of p
    num_p = length(p);
    p_inside = [];
    p_inside_index = [];
    for i=1:num_p
        
        p_ = p(i,1:2);
        vec_area_p = [];
        for n=1:(num_area-1)
            vec_area_p = [vec_area_p; p_ - area_(n,:)];
        end
        
        % signal between the 2 vectors
        signal = [];
        for n=1:(num_area-1)
            signal = [signal; dot(vec_area_p(n,:), vec_area(n,:))];
        end
        
        ind_aux = 1:length(signal);        
        if isempty(ind_aux(signal<0))
           p_inside = [p_inside; p(i,:)];
           p_inside_index = [p_inside_index; i]; 
        end
                
    end
    
end
        