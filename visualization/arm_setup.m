function [link] = arm_setup()

    ralink    = struct('name', 'base', 'offset', [ 0.0 0.0 0.00]', 'child', 2, 'dir', [0 1 0]', 'angle', 90, 'pos', [0,0,0], 'rot', eye(3));
    ralink(2) = struct('name', '#1',   'offset', [ 0.0 0.0 10]', 'child', 3, 'dir', [1 0 0]', 'angle', 0, 'pos', [0,0,0], 'rot', eye(3));
    ralink(3) = struct('name', '#2',   'offset', [ 0.0 0.0 5]', 'child', 4, 'dir', [0 1 0]', 'angle', 0, 'pos', [0,0,0], 'rot', eye(3));
    ralink(4) = struct('name', '#3',   'offset', [ 0.0 0.0 5]', 'child', 0, 'dir', [0 1 0]', 'angle', 0, 'pos', [0,0,0], 'rot', eye(3));

    link = fk(ralink);

    