%% Ground-truth sphere fit on STL vertices (mm)
fruit_files = {'apple.stl','orange.stl','strawberry.stl','peach.stl','pear.stl'};
fruit_names = {'apple','orange','strawberry','peach','pear'};

fprintf('\n%-12s %10s %10s %10s %10s\n','fruit','sphere_mm','rms_mm','bbox_max','bbox_min');
for i = 1:numel(fruit_files)
    tri = stlread(fruit_files{i});
    V = double(tri.Points);                      % mm

    % --- algebraic (Kasa) initial guess ---
    A = [2*V, ones(size(V,1),1)];
    b = sum(V.^2,2);
    sol = A\b;
    x = [sol(1:3)', sqrt(sol(4) + sol(1:3)'*sol(1:3))];   % [cx cy cz r]

    % --- geometric (orthogonal-distance) refinement, Gauss-Newton ---
    for k = 1:100
        d = V - x(1:3);
        n = sqrt(sum(d.^2,2));
        res = n - x(4);
        J = [-d./n, -ones(size(V,1),1)];
        dx = -(J\res);
        x = x + dx';
        if norm(dx) < 1e-9, break; end
    end
    c = x(1:3); r = x(4);
    rms = sqrt(mean((sqrt(sum((V-c).^2,2)) - r).^2));

    ext = max(V) - min(V);
    fprintf('%-12s %10.2f %10.2f %10.2f %10.2f\n', ...
        fruit_names{i}, 2*r, rms, max(ext), min(ext));
end