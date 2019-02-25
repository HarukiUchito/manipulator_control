function [link] = fk(link)

    idx = 1;
    link(idx).rot = dp_get_rpy_rot(link(idx).dir * link(idx).angle);
    link(idx).pos = link(idx).offset;

    while link(idx).child > 0
        parent = idx;
        idx = link(idx).child;
        %cidx = link(idx).child;

        link(idx).rot = dp_get_rpy_rot(link(idx).dir * link(idx).angle);
        link(idx).rot = link(parent).rot * link(idx).rot;

        link(idx).pos = link(parent).pos + link(idx).rot * link(idx).offset;    
    end
end



