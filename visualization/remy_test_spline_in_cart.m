clear all

cnc_obj = Connection_gen;

cnc_obj.open();

cnc_obj.calcTrajectory();

fileID = fopen('../param_files/input.in','r');
formatSpec = '%f %f %f %f';
sizeP = [4 Inf];
P = fscanf(fileID,formatSpec,sizeP);

link = arm_setup();

link_idx = size(link, 2);
offset = [0,0,0];

format long
traj = [0,0,0];

while 0 <= cnc_obj.receive([link(2:4).angle])

    R_IK = cnc_obj.send();

    link = set_jangles(link, R_IK);
    link = fk(link);

    dp_draw_links(link, [1,1,1]);

    view(100.0, 7.264389682754654);
    cur_pos = positions(link, link_idx, offset);
    plot3(cur_pos(1), cur_pos(2), cur_pos(3), 'o');
    traj = [traj; [cur_pos(1), cur_pos(2), cur_pos(3)]];
    plot3(traj(:,1), traj(:,2), traj(:,3), 'x');

    % draw given points
    scatter3(P(1,:), P(2,:), P(3,:))

    grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    pause(1 / 50);
end

clear cnc_obj
clear functions