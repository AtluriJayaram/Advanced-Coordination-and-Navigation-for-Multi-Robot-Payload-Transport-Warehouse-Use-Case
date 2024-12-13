% Market-based coordination for controlled transport of multiple objects by robots
clc; clear; close all;

% Parameters
total_robots = 5;
box_positions = [5, 5; 10, 20; 48, 10; 5, 40];
target_positions = [20, 20; 30, 30; 45, 45; 30, 10];
box_weights = [40, 60, 80, 30];  % Different weights for each box
robot_speed = 0.5;
max_iterations = 500;
robot_capacity = 10;

% Initialize grid-based warehouse
grid_size = [50, 50];
grid_state = zeros(grid_size);

% Define container locations and obstacles
containers_loc = struct('start_x', {5, 20, 35, 5, 20, 35}, ...
                        'start_y', {10, 10, 10, 30, 30, 30}, ...
                        'width',   {5, 5, 5, 5, 5, 5}, ...
                        'height',  {8, 8, 8, 8, 8, 8});

% Generate container coordinates and append to obstacles
obstacles = [];
for i = 1:length(containers_loc)
    coords = generateContainerCoords(containers_loc(i).start_x, containers_loc(i).start_y, ...
        containers_loc(i).width, containers_loc(i).height);
    obstacles = [obstacles; coords];
end

% Add obstacles to the grid
for obs = 1:size(obstacles, 1)
    grid_state(obstacles(obs, 2), obstacles(obs, 1)) = 1;
end

% Initialize robot positions randomly without placing them on obstacles
robot_positions = zeros(total_robots, 2);
for i = 1:total_robots
    while true
        x = randi(grid_size(2));
        y = randi(grid_size(1));
        if grid_state(y, x) == 0
            robot_positions(i, :) = [x, y];
            grid_state(y, x) = 1;
            break;
        end
    end
end

% Visualization setup
figure;
hold on;
axis([0 grid_size(2) 0 grid_size(1)]);
set(gca, 'xtick', [], 'ytick', []);
title('Market-Based Robot Coordination');
xlabel('X Position');
ylabel('Y Position');

% Create hash map for rectangles
rect_handles = containers.Map('KeyType', 'char', 'ValueType', 'any');
for x = 1:grid_size(2)
    for y = 1:grid_size(1)
        key = sprintf('%d,%d', x, y);
        rect_handles(key) = rectangle('Position', [x - 1, y - 1, 1, 1], ...
            'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'k');
    end
end

% Color coding
empty_color = [0.9 0.9 0.9];
robot_color = [0 0 1];
box_color = [0 0 0];
target_color = [1 0 0];
obstacle_color = [0.4 0.4 0.4];

% Set initial colors
updateColor(rect_handles, box_positions, box_color);
updateColor(rect_handles, target_positions, target_color);
updateColor(rect_handles, obstacles, obstacle_color);
updateColor(rect_handles, robot_positions, robot_color);

% Concurrent Task Execution
% Use a timer to mimic asynchronous execution
tasks_completed = false(size(box_positions, 1), 1);
timer = tic;

while ~all(tasks_completed)
    % Main simulation loop, assuming all variables are initialized and set up properly.
    for task = 1:size(box_positions, 1)
        % Calculate the number of robots required based on box weight
        required_robots = ceil(box_weights(task) / robot_capacity);

        if (required_robots > total_robots)
            disp(['Not enough robots to perform task ' num2str(task) '. Needed: ' num2str(required_robots) ', Available: ' num2str(total_robots)]);
            continue; % Skip this task as it can't be performed
        end

        if ~tasks_completed(task)  % Assuming tasks_completed is a logical array tracking task completion
            [is_done, robot_positions, box_positions] = performTask(task, robot_positions, box_positions, target_positions, ...
                grid_state, max_iterations, robot_speed, obstacles, grid_size, ...
                required_robots, rect_handles, robot_color, box_color, ...
                target_color, obstacle_color, empty_color);
            tasks_completed(task) = is_done;
        end
        % Update visualization if needed here
    end

    % Update visualization
    updateVisualization(rect_handles, robot_positions, box_positions, target_positions, obstacles, ...
                        robot_color, box_color, target_color, obstacle_color, empty_color);
    pause(0.01); % Slow down the loop for visualization purposes
end

elapsed_time = toc(timer);
disp(['All tasks completed in ', num2str(elapsed_time), ' seconds!']);


%%
% Functions
function updated_grid = expand_obstacles(grid_state, size_increase)
    % Initialize the updated grid as a copy of the original grid state
    updated_grid = grid_state;

    % Get the size of the grid
    [grid_rows, grid_cols] = size(grid_state);

    % Loop through each cell in the grid
    for x = 1:grid_cols
        for y = 1:grid_rows
            % Check if the current cell contains an obstacle
            if grid_state(y, x) == 1
                % Calculate the expansion range around the cell
                x_min = max(1, x - size_increase);
                x_max = min(grid_cols, x + size_increase);
                y_min = max(1, y - size_increase);
                y_max = min(grid_rows, y + size_increase);

                % Set the cells within the expansion range to 1
                for xi = x_min:x_max
                    for yi = y_min:y_max
                        updated_grid(yi, xi) = 1;
                    end
                end
            end
        end
    end
end

function [is_done, robot_positions, box_positions] = performTask(task, robot_positions, box_positions, target_positions, grid_state, max_iterations, robot_speed, obstacles, grid_size, required_robots, rect_handles, robot_color, box_color, target_color, obstacle_color, empty_color)
    first_time_printing = true;
    % Retrieve positions
    box_position = box_positions(task, :);
    target_position = target_positions(task, :);

    % Bidding and assignment
    total_robots = size(robot_positions, 1);
    robot_bids = zeros(total_robots, 1);
    for i = 1:total_robots
        distance_to_box = norm(robot_positions(i, :) - box_position);
        robot_bids(i) = 1 / distance_to_box;  % Closer robots have higher priority
    end
    [~, sorted_indices] = sort(robot_bids, 'descend');
    selected_robots = sorted_indices(1:min(required_robots, total_robots));

    % Calculate assigned positions dynamically around the box
    assigned_positions = calculateAssignedPositions(box_position, required_robots, grid_size);

    % Movement simulation
    box_moving = false;
    is_done = false;
    for iter = 1:max_iterations
        pause(0.01); % Slow down the loop for visualization

        if ~box_moving
            % Phase 1: Robots moving to box
            for i = 1:length(selected_robots)
                robot_idx = selected_robots(i);
                start = robot_positions(robot_idx, :);
                goal = assigned_positions(i, :);

                temp_grid = update_grid_state(grid_size, robot_positions, box_positions, obstacles);
                temp_grid(start(2), start(1)) = 0;  % Make current robot's position traversable

                path = astar_cardinal(temp_grid, start, goal);
                if ~isempty(path) && size(path, 1) > 1
                    new_position = path(2, :);
                    robot_positions(robot_idx, :) = new_position;
                end
            end

            % Check if all robots are in position
            all_in_place = true;
            for i = 1:length(selected_robots)
                robot_idx = selected_robots(i);
                if norm(robot_positions(robot_idx, :) - assigned_positions(i, :)) > 0.5
                    all_in_place = false;
                    break;                
                end
            end

            if all_in_place
                box_moving = true;
                % Convert the robot indices to a string for display
                robot_indices_str = num2str(selected_robots(:)');
                % Display the message with robot indices and task number
                disp(['Robots [' robot_indices_str '] successfully reached the target box ' num2str(task) '!']);
            end

        else
            % Phase 2: Group movement
            temp_robot_positions = robot_positions;
            temp_robot_positions(selected_robots, :) = [];  % Remove selected robots
            temp_box_positions = box_positions;
            temp_box_positions(task, :) = [];  % Remove current box
            temp_grid = update_grid_state(grid_size, temp_robot_positions, temp_box_positions, obstacles);
            group_radius=2;
            temp_grid = expand_obstacles(temp_grid, group_radius-1);
            group_path = astar_cardinal(temp_grid, box_position, target_position);

            if ~isempty(group_path) && size(group_path, 1) > 1
                if first_time_printing
                    first_time_printing = false;
                    disp(['Robots on the move! Transporting the target box ' num2str(task) '...']);
                end
                new_box_position = group_path(2, :);

                assigned_positions = calculateAssignedPositions(new_box_position, length(selected_robots), grid_size);

                for i = 1:length(selected_robots)
                    robot_positions(selected_robots(i), :) = assigned_positions(i, :);
                end

                box_position = new_box_position;
                box_positions(task, :) = box_position;
            else
                disp(['Robots [' robot_indices_str '] cannot reach provided destination for the target box ' num2str(task) '!']);
                break;
            end

            % Check if the box has reached the target
            if all(box_position == target_position)
                is_done = true;
                disp(['Box ' num2str(task) ' successfully transported to the target position!']);
            end
        end

        % Update visualization
        updateVisualization(rect_handles, robot_positions, box_positions, target_positions, obstacles, robot_color, box_color, target_color, obstacle_color, empty_color);
    end
end

function grid_state = update_grid_state(grid_size, robot_positions, box_positions, obstacles)
    grid_state = zeros(grid_size);
    
    if ~isempty(obstacles)
        for i = 1:size(obstacles, 1)
            grid_state(obstacles(i, 2), obstacles(i, 1)) = 1;
        end
    end
    
    if ~isempty(robot_positions)
        for i = 1:size(robot_positions, 1)
            grid_state(robot_positions(i, 2), robot_positions(i, 1)) = 1;
        end
    end
    
    if ~isempty(box_positions)
        for i = 1:size(box_positions, 1)
            grid_state(box_positions(i, 2), box_positions(i, 1)) = 1;
        end
    end
end

function dist = manhattan_distance(a, b)
    dist = abs(a(1) - b(1)) + abs(a(2) - b(2));
end

function path = reconstruct_path(came_from, goal_x, goal_y)
    path = [goal_x, goal_y];
    current_x = goal_x;
    current_y = goal_y;
    while any(came_from(current_y, current_x, :))
        [current_x, current_y] = deal(came_from(current_y, current_x, 1), came_from(current_y, current_x, 2));
        path = [current_x, current_y; path];
    end
end

function updateVisualization(rect_handles, robot_positions, box_positions, target_positions, obstacles, robot_color, box_color, target_color, obstacle_color, empty_color)
    keys = rect_handles.keys;
    for k = 1:length(keys)
        key = keys{k};
        [x, y] = parseKey(key);
        color = empty_color;
        
        if any(robot_positions(:, 1) == x & robot_positions(:, 2) == y)
            color = robot_color;
        elseif any(box_positions(:, 1) == x & box_positions(:, 2) == y)
            if any(all([box_positions(:, 1) == x & box_positions(:, 2) == y], 2) & all([target_positions(:, 1) == x & target_positions(:, 2) == y], 2))
                color = [0 0.8 0];  % Green color for box at its target
            else
                color = box_color;
            end
        elseif any(target_positions(:, 1) == x & target_positions(:, 2) == y)
            color = target_color;
        elseif ~isempty(obstacles) && any(obstacles(:, 1) == x & obstacles(:, 2) == y)
            color = obstacle_color;
        end
        
        set(rect_handles(key), 'FaceColor', color);
    end
    drawnow;
end

function coords = generateContainerCoords(start_x, start_y, width, height)
    coords = [];
    for dx = 0:(width-1)
        for dy = 0:(height-1)
            x = start_x + dx;
            y = start_y + dy;
            coords = [coords; x, y];
        end
    end
end

function assigned_positions = calculateAssignedPositions(box_position, required_robots, grid_size)
    assigned_positions = zeros(required_robots, 2);
    angle_increment = 2 * pi / required_robots;
    radius = 1;  % Fixed radius for simplicity
    for i = 1:required_robots
        angle = (i-1) * angle_increment;
        x_offset = round(radius * cos(angle));
        y_offset = round(radius * sin(angle));
        assigned_positions(i, :) = box_position + [x_offset, y_offset];
    end
end

function path = astar_cardinal(grid_state, start, goal)
    [rows, cols] = size(grid_state);
    closed_set = false(rows, cols);
    came_from = zeros(rows, cols, 2);
    g_score = inf(rows, cols);
    f_score = inf(rows, cols);

    % Initialize start and goal positions
    start_x = start(1);
    start_y = start(2);
    goal_x = goal(1);
    goal_y = goal(2);

    % Priority queue: use a MATLAB cell array with custom sorting
    open_set = {};
    open_set{end + 1} = struct('f', 0, 'x', start_x, 'y', start_y);

    g_score(start_y, start_x) = 0;
    f_score(start_y, start_x) = manhattan_distance([start_x, start_y], [goal_x, goal_y]);

    while ~isempty(open_set)
        % Find the node with the lowest f_score
        [~, idx] = min(cellfun(@(node) node.f, open_set));
        current = open_set{idx};
        open_set(idx) = []; % Remove the current node from open set

        current_x = current.x;
        current_y = current.y;

        % Check if the goal is reached
        if current_x == goal_x && current_y == goal_y
            path = reconstruct_path(came_from, goal_x, goal_y);
            return;
        end

        closed_set(current_y, current_x) = true;

        % Explore neighbors
        neighbors = [
            0, 1;  % Up
            0, -1; % Down
            1, 0;  % Right
            -1, 0  % Left
        ];

        for i = 1:size(neighbors, 1)
            neighbor_x = current_x + neighbors(i, 1);
            neighbor_y = current_y + neighbors(i, 2);

            % Skip out-of-bounds, obstacles, or already visited cells
            if neighbor_x < 1 || neighbor_x > cols || neighbor_y < 1 || neighbor_y > rows || ...
               grid_state(neighbor_y, neighbor_x) == 1 || closed_set(neighbor_y, neighbor_x)
                continue;
            end

            % Calculate tentative g-score
            tentative_g_score = g_score(current_y, current_x) + 1;

            % If a better path is found, update records
            if tentative_g_score < g_score(neighbor_y, neighbor_x)
                came_from(neighbor_y, neighbor_x, :) = [current_x, current_y];
                g_score(neighbor_y, neighbor_x) = tentative_g_score;
                f_score(neighbor_y, neighbor_x) = tentative_g_score + manhattan_distance([neighbor_x, neighbor_y], [goal_x, goal_y]);

                % Add neighbor to the open set if not already present
                if ~any(cellfun(@(node) node.x == neighbor_x && node.y == neighbor_y, open_set))
                    open_set{end + 1} = struct('f', f_score(neighbor_y, neighbor_x), 'x', neighbor_x, 'y', neighbor_y);
                end
            end
        end
    end

    % If no path is found, return an empty path
    path = [];
end

function [x, y] = parseKey(key)
    coords = sscanf(key, '%d,%d');
    x = coords(1);
    y = coords(2);
end

function updateColor(rect_handles, positions, color)
    for i = 1:size(positions, 1)
        key = sprintf('%d,%d', positions(i, 1), positions(i, 2));
        if isKey(rect_handles, key)
            set(rect_handles(key), 'FaceColor', color);
        end
    end
end
