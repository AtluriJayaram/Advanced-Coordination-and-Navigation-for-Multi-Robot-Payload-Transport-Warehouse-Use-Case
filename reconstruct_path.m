function path = reconstruct_path(came_from, goal_x, goal_y)
    path = [goal_x, goal_y];
    current_x = goal_x;
    current_y = goal_y;
    while any(came_from(current_y, current_x, :))
        [current_x, current_y] = deal(came_from(current_y, current_x, 1), came_from(current_y, current_x, 2));
        path = [current_x, current_y; path];
    end
end