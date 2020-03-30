% Function to find roots using brute-force method. 
% Version with preallocation.
function all_roots = brute_force_root_finder_preallocate_1(f, a, b, n)
    x = linspace(a, b, n);
    y = f(x);
    roots = zeros(length(x), 1);
    r_cnt = 0;
    for i = 1:(n-1)
        if y(i)*y(i+1) < 0
            root = x(i) - (x(i+1) - x(i))/(y(i+1) - y(i))*y(i);
            % roots = [roots; root];
            r_cnt = r_cnt + 1;
            roots(r_cnt) = root;
        end
    end
    all_roots = roots(1:r_cnt);
end