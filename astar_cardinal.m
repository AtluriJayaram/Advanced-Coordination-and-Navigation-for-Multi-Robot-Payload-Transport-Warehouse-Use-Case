function path = astar_cardinal(grid_state, start, goal)
    [rows, cols] = size(grid_state);
    closed_set = false(rows, cols);
    open_set = false(rows, cols);
    came_from = zeros(rows, cols, 2);
    g_score = inf(rows, cols);
    f_score = inf(rows, cols);

    start_x = start(1);
    start_y = start(2);
    goal_x = goal(1);
    goal_y = goal(2);

    open_set(start_y, start_x) = true;
    g_score(start_y, start_x) = 0;
    f_score(start_y, start_x) = manhattan_distance([start_x, start_y], [goal_x, goal_y]);

    while any(open_set(:))
        [~, current] = min(f_score(open_set));
        [current_y, current_x] = ind2sub(size(grid_state), find(open_set, 1));
        
        if current_x == goal_x && current_y == goal_y
            path = reconstruct_path(came_from, goal_x, goal_y);
            return;
        end
        
        open_set(current_y, current_x) = false;
        closed_set(current_y, current_x) = true;

        for i=1:4
            switch i
                case 1
                    dx = 0; dy = 1;
                case 2
                    dx = 0; dy = -1;
                case 3
                    dx = 1; dy = 0;
                case 4
                    dx = -1; dy = 0;
            end
            
            neighbor_x = current_x + dx;
            neighbor_y = current_y + dy;

            if neighbor_x < 1 || neighbor_x > cols || neighbor_y < 1 || neighbor_y > rows || ...
               grid_state(neighbor_y, neighbor_x) == 1 || closed_set(neighbor_y, neighbor_x)
                continue;
            end

            tentative_g_score = g_score(current_y, current_x) + 1;
            
            if ~open_set(neighbor_y, neighbor_x)
                open_set(neighbor_y, neighbor_x) = true;
            elseif tentative_g_score >= g_score(neighbor_y, neighbor_x)
                continue;
            end

            came_from(neighbor_y, neighbor_x, :) = [current_x, current_y];
            g_score(neighbor_y, neighbor_x) = tentative_g_score;
            f_score(neighbor_y, neighbor_x) = tentative_g_score + manhattan_distance([neighbor_x, neighbor_y], [goal_x, goal_y]);
        end
    end
    path = [];
end