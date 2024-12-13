function path = a_star(grid, start, goal, heuristic)
    % Convert start and goal to linear indices
    startIdx = sub2ind(size(grid), start(1), start(2));
    goalIdx = sub2ind(size(grid), goal(1), goal(2));

    % Initialize open and closed sets
    openSet = [startIdx];
    cameFrom = zeros(size(grid));

    % Cost from start along best known path.
    gScore = inf(size(grid));
    gScore(startIdx) = 0;

    % Estimated total cost from start to goal through y.
    fScore = inf(size(grid));
    fScore(startIdx) = heuristic(start, goal);

    while ~isempty(openSet)
        % Current node in open set having the lowest fScore[] value
        [~, idx] = min(fScore(openSet));
        currentIdx = openSet(idx);
        
        if currentIdx == goalIdx
            path = reconstruct_path(cameFrom, goalIdx, size(grid));
            return;
        end
        
        openSet(idx) = []; % Remove current from open set
        current = ind2sub(size(grid), currentIdx);
        neighbors = get_neighbors(current, grid);
        
        for i = 1:size(neighbors, 1)
            neighborIdx = sub2ind(size(grid), neighbors(i,1), neighbors(i,2));
            
            if grid(neighbors(i,1), neighbors(i,2)) == 1 % Obstacle check
                continue;
            end
            
            tentative_gScore = gScore(currentIdx) + 1;
            if tentative_gScore < gScore(neighborIdx)
                % This path to neighbor is better than any previous one. Record it!
                cameFrom(neighborIdx) = currentIdx;
                gScore(neighborIdx) = tentative_gScore;
                fScore(neighborIdx) = gScore(neighborIdx) + heuristic(neighbors(i,:), goal);
                if ~ismember(neighborIdx, openSet)
                    openSet(end+1) = neighborIdx; % Discover a new node
                end
            end
        end
    end

    % If we get here, then no path was found
    path = [];
end

function path = reconstruct_path(cameFrom, currentIdx, gridSize)
    % Reconstruct path back to the start using indices
    path = [];
    while cameFrom(currentIdx) ~= 0
        [currentX, currentY] = ind2sub(gridSize, currentIdx);
        path = [[currentX, currentY]; path];
        currentIdx = cameFrom(currentIdx);
    end
end

function neighbors = get_neighbors(current, grid)
    [x, y] = deal(current(1), current(2));
    neighbors = [];
    for dx = -1:1
        for dy = -1:1
            nx = x + dx;
            ny = y + dy;
            if dx == 0 && dy == 0
                continue; % Skip the current node
            end
            if nx > 0 && nx <= size(grid, 1) && ny > 0 && ny <= size(grid, 2)
                if grid(nx, ny) == 0 % Only consider non-obstacle spaces
                    neighbors = [neighbors; nx, ny];
                end
            end
        end
    end
end

function d = euclidean(pos1, pos2)
    % Euclidean distance heuristic for A* pathfinding
    d = sqrt(sum((pos1 - pos2) .^ 2));
end
