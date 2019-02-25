function dp_draw_links(link, view_)

    clf;
    %view([1 0.5 0.2]); 

    if nargin < 2
        view_ = [1,1,1];
    end
    view(view_);

    axis equal;
    grid on;
    hold on;
    
    %xlim([-0.2, 0.8]);
    %ylim([-0.5, 0.5]);
    %zlim([-0.2, 0.8]);
    
% ee_type (end efector type) : 'hand' or 'foot' or 'head'

    % 0.040, 0.070
    j_radius = 0.3;
    j_len    = 1.1;

    idx = 1;
    link(idx).rot = dp_get_rpy_rot(link(idx).dir * link(idx).angle);
    link(idx).pos = link(idx).offset;

    while link(idx).child > 0
        parent = idx;
        idx = link(idx).child;
        cidx = link(idx).child;

        from = link(parent).pos;
        to   = link(idx).pos;
        plot3([from(1) to(1)], [from(2), to(2)], [from(3), to(3)], 'LineWidth', 5);
%
        loffset = link(idx).offset;
        % link1
        if norm([loffset(1), loffset(2)]) > 30
            dir    = link(idx).rot * [loffset(1), loffset(2), 0.0]';
            dir_len= norm(dir);
            ndir    = dir / dir_len;
            %draw_cylinder(from + dir/2.0, ndir, 20, dir_len, 6);
        end
        % link2 
        offset = link(idx).rot * [loffset(1), loffset(2), loffset(3)/ 2.0]';
        len = [0,0,j_len/4.0] * link(idx).dir;
        joint_offset = link(idx).rot * [0,0,1]' * len;

        cjoint_offset = [0.0,0.0,0.0]';
        if cidx > 0
            cjoint_offset_len = [0,0,j_len/4.0] * link(cidx).dir;
            cjoint_offset = dp_get_rpy_rot(link(cidx).dir * link(cidx).angle) * [0,0,1]' * cjoint_offset_len;
        end

        if link(idx).child == 0
            radius = 0.25;
        else 
            radius = 0.5;
        end
        draw_cylinder(from + offset + joint_offset - cjoint_offset, link(idx).rot, link(idx).rot * [0,0,1]', radius, norm(link(idx).offset) - norm(joint_offset)*2.0 - norm(cjoint_offset)*2.0, 10, 'white');
        % joint
        if link(idx).dir == [1,0,0]'
            rot2cylrot = dp_get_rpy_rot([0,90,0]);
        elseif link(idx).dir == [0,1,0]'
            rot2cylrot = dp_get_rpy_rot([90,0,0]);
        else
            rot2cylrot = dp_get_rpy_rot([0,0,90]);
        end
        draw_cylinder(from,  link(idx).rot * rot2cylrot, link(idx).rot * link(idx).dir, j_radius, j_len, 10, 'white');
%}
    end

    % light();
    light('Position', [2,2,2]);
    xlabel('x');
    ylabel('y');
    zlabel('z');
end



