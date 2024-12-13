function [dx, dy] = move_cardinal(direction)
    if abs(direction(1)) > abs(direction(2))
        dx = sign(direction(1));
        dy = 0;
    else
        dx = 0;
        dy = sign(direction(2));
    end
end