% Assignment 2
% Kiana Bronder (kbronde1)

function Rx = solveRx( alphas, betas ) 

% function description
% alphas: A 3xN matrix representing the skew symmetric matrices. That is,
% 𝑎𝑎𝑎𝑎𝑎𝑎ℎ𝑎𝑎𝑎𝑎 = [𝛼1  ⋯ 𝛼𝑁] 
% betas: A 3xN matrix representing the skew symmetric matrices. That is,
% 𝑏𝑏𝑏𝑏𝑡𝑡𝑎𝑎𝑎𝑎 = [𝛽1 ⋯ 𝛽𝑁] 
% return: The least squares solution to the matrix Rx.

M = zeros(3);
for i = 1:size(alphas,2)
    M = M + betas(:,i) * transpose(alphas(:,i));
end
Rx = (transpose(M) * M)^(-1/2) * transpose(M);

end