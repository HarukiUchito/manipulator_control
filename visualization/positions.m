function [pos] = positions(link, r_idx, offset)

    % control point
    p = link(r_idx).pos + link(r_idx).rot * offset';
    r = link(r_idx).rot * offset';
    pos = [p;r];

end



