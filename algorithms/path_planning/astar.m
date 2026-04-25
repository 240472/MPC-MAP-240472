function [path] = astar(read_only_vars, public_vars)

    map   = read_only_vars.discrete_map.map;
    goal  = read_only_vars.discrete_map.goal;

    % kernel = [0 0 1 0 0;
    %       0 8 10 8 0;
    %       1 10 100 10 1;
    %       0 8 10 8 0;
    %       0 0 1 0 0]*100;

    
    % dilatation for diagonal walls
    inf_kernel = [0 1 0;
                  1 1 1;
                  0 1 0];

    inf_mask = imdilate(map == 1, inf_kernel);
    
    % convolution for danger zones near walls
    kernel =   [0 0 0 1 0 0 0
                0 0 1 2 1 0 0
                0 1 2 3 2 1 0
                1 2 3 4 3 2 1
                0 1 2 3 2 1 0
                0 0 1 2 1 0 0
                0 0 0 1 0 0 0]*1;

    cost_map = conv2(map, kernel, 'same');
    cost_map(inf_mask) = inf;

    % something like this will be there in the final app
    pose  = public_vars.estimated_pose(1:2);

    % temporary solution to start position
    pose = [1,1]; 
    start = pose/read_only_vars.map.discretization_step;
    
    path  = [];

    [rows, cols] = size(map);

    sr = start(2);  % start row
    sc = start(1);  % start collum           
    gr = goal(2);   % goal row      
    gc = goal(1);   % goal collum
    

    % handling floating points
    sr = round(sr);  sc = round(sc);
    gr = round(gr);  gc = round(gc);

    % init
    g_mat      = inf(rows, cols);
    dead_list = zeros(rows, cols);
    parent_r   = zeros(rows, cols);
    parent_c   = zeros(rows, cols);

    g_mat(sr, sc) = 0;

    % alive list : [f, g, row, col, parent_row, parent_col]
    alive_list = [0, 0, sr, sc, sr, sc];

    found = false;

    % main loop
    while size(alive_list, 1) > 0

        % pick a cell with lowest travel cost
        [~, idx] = min(alive_list(:, 1));
        current_node = alive_list(idx, :);
        alive_list(idx, :) = [];

        cr = current_node(3);       % current row
        cc = current_node(4);       % current collum

        if dead_list(cr, cc) == 1
            continue
        end
        dead_list(cr, cc) = 1;

        parent_r(cr, cc) = current_node(5);
        parent_c(cr, cc) = current_node(6);

        % found goal
        if cr == gr && cc == gc
            found = true;
            break
        end

        % 8-connected neighbours
        neighbours = [cr-1, cc;       
                      cr+1, cc;                 
                      cr,   cc-1;     
                      cr,   cc+1;     
                      cr-1, cc-1;     
                      cr-1, cc+1;     
                      cr+1, cc-1;     
                      cr+1, cc+1];    

        for i = 1:8
            nr = neighbours(i, 1);      % neigbour row
            nc = neighbours(i, 2);      % neighbour collum

            if nr < 1 || nr > rows || nc < 1 || nc > cols
                continue
            end
            
            % handling diagonal and up/down/left/right travel costs
            if nr ~= cr && nc ~= cc
                step_cost = sqrt(2);
            else
                step_cost = 1;
            end
        
            % final travel cost
            new_g = g_mat(cr, cc) + step_cost + cost_map(nr, nc);

            % updating if better cost is reached
            if new_g < g_mat(nr, nc)
                g_mat(nr, nc) = new_g;
                %new_h = max(abs(nr - gr), abs(nc - gc));   % Chebyshev distance
                new_h = sqrt((nr - gr)^2 + (nc - gc)^2);    % Euler distance
                new_f = new_g + new_h;
                alive_list(end+1, :) = [new_f, new_g, nr, nc, cr, cc];
            end
        end
    end

    % reconstruct path in [x, y] coordinates
    if found
        r = gr;
        c = gc;
        while ~(r == sr && c == sc)
            x = c - 1;      % MATLAB SHENANIGANS
            y = r - 1;
            path = [x, y; path];
            pr = parent_r(r, c);
            pc = parent_c(r, c);
            r  = pr;
            c  = pc;
        end
        
        % scaling back
        path = [start(1), start(2); path]*read_only_vars.map.discretization_step;
    
    end

    if isempty(path)
        warning("Path was not generated!")
    end

end