function demo_brute_force_root_finder_preallocation_test()
f = @(x) exp(-x.^2).*cos(20*x) + cos(20*x);
a = 0; b = 1000; n = 100100;
disp('without preallocate')
tic
 for i = 1:1000
    roots = brute_force_root_finder(...
        f, a, b, n);
 end
toc


disp('preallocate 1')
tic
for i = 1:1000
    roots = brute_force_root_finder_preallocate_1(...
        f, a, b, n);
 end
toc

disp('preallocate 2')
tic
 for i = 1:1000
    roots = brute_force_root_finder_preallocate_2(...
        f, a, b, n,50000);
 end
toc

disp('preallocate lut')

tic
 for i = 1:1000
    roots = brute_force_root_finder_preallocate_lut(...
        f, a, b, n);
 end
toc

if ~isempty(roots)
%     roots
    disp(numel(roots))
    x = linspace(a, b, n);
    plot(x, f(x), roots, zeros(size(roots)), 'or')
else
    disp('Could not find any roots');
end

end