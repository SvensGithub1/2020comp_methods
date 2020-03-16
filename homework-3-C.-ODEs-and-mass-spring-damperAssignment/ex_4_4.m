%{
the purpose of this script is to numerical solution the logistic equation.
And find a acceptable result by visually compare two plots
%}
clear
close all
f = @(u, t) 0.1*(1 - u/500)*u;
U_0 = 100;
T = 60;
continue_iteration = true;
delta_t = 10;
dt_k_prev = delta_t;
[u_k_prev, t_k_prev] = ode_FE(f, U_0, dt_k_prev, T);


dt_k = 0;
u_k = 0;
t_k = 0;
k=0;
while continue_iteration %continiue until user finds result satisfactory
    figure(1)
    k=k+1;
    dt_k = 2^(-k) * delta_t;
    [u_k, t_k] = ode_FE(f, U_0, dt_k, T);
    
    plot(t_k_prev, u_k_prev, 'b-','LineWidth',2);
    xlabel('t'); ylabel('N(t)');
    hold on
    plot(t_k, u_k, 'r--','LineWidth',2);
    xlabel('t'); ylabel('N(t)');
    legend(['{\Delta}t_k_-_1=',num2str(dt_k_prev)],['{\Delta}t_k=',num2str(dt_k)],'Location','northwest')
    title('Numerical Solution of Logistic Equation')
    
    
    while true
        disp('Please compare the plotted lines and decide if the iteration should continue.');
    user_input =input('Continue the iteration process [y/n]: ','s');
        if strcmp('n', user_input)
            continue_iteration = false;
            disp('The ideration has ended.')
            break
        elseif strcmp('y', user_input) %k becomes k-1
            dt_k_prev = dt_k;
            u_k_prev = u_k;
            t_k_prev = t_k;
            break
        else
            disp('please provide valid answer')
            
        end
    end
    hold off
end

% Note: this print statement gets a problem with the decimal point
% so we rather do it like this:
dt_k_prev_str = strrep(num2str(dt_k_prev),'.','_');
dt_k_str = strrep(num2str(dt_k),'.','_');
filename = strcat('ex_4_4_dt_k-1=',dt_k_prev_str,'_dt_k=',dt_k_str);

print(filename, '-dpng');

print(filename, '-dpdf');
