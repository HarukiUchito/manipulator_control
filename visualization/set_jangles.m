function [rlink] = set_jangles(link, angles)

    r_idx = size(angles, 1);

    i = 1;
    idx = 1;
    while idx <= r_idx        
        parent = idx;
        idx = link(idx).child;
        cidx = link(idx).child;
        
        link(idx).angle = rad2deg(angles(i));

        i = i + 1;
    end
    
    rlink = link;
end



