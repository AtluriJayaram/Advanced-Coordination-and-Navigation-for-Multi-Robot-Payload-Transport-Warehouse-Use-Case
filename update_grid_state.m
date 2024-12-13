function grid_state = update_grid_state(grid_size, robot_positions, box_position, obstacles)
    grid_state = zeros(grid_size);
    
    % Mark obstacles if they exist
    if ~isempty(obstacles)
        for i = 1:size(obstacles, 1)
            grid_state(obstacles(i, 2), obstacles(i, 1)) = 1;
        end
    end
    
    % Mark robots if they exist
    if ~isempty(robot_positions)
        for i = 1:size(robot_positions, 1)
            grid_state(robot_positions(i, 2), robot_positions(i, 1)) = 1;
        end
    end
    
    % Mark box if it exists
    if ~isempty(box_position)
        grid_state(box_position(2), box_position(1)) = 1;
    end
end