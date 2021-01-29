clc                                 % To clear the command window
clear                               % To clear the workspace
%% Problem settings
lb = [-1,-1,5,pi/4];                         % Lower bound
ub = [1,1,6,pi/3];                           % Upper bound
prob = @objfun;                  % Fitness function
%% Parameters for Differential Evolution
Np = 200;                             % Population Size
T = 5000;                             % No. of iterations
Pc = 0.9;                           % crossover probability
F = 0.6;                           % Scaling factor
rng(1,'twister')
[bestsol,bestfitness] = DifferentialEvolution(prob,lb,ub,Np,T,Pc,F)
%% Calling the best solution to get complete link values
X = bestsol;
[err,link] = objfun(X);
link