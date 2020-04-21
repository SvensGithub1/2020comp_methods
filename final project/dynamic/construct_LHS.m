function LHS = construct_LHS(M,Cq_fun,t_0, q_0,  bodies)

number_coordinates = 3*length(bodies);
LHS = zeros(size(M)+size(Cq_fun) + [number_coordinates number_coordinates]);
LHS(1:size(M,1),1:size(M,2)) = M;
LHS(1:size(Cq_fun,2),size(M,2)+1:size(M,2)+size(Cq_fun,1)) = Cq_fun';
LHS(size(M,1)+1:size(M,1)+size(Cq_fun,1),1:size(Cq_fun,2)) = Cq_fun;
LHS(end-number_coordinates + 1:end,end-number_coordinates + 1:end) = eye(number_coordinates);