% Assignment 2
% Kiana Bronder (kbronde1)

function tx = solveTx( RA, tA, RB, tB, RX ) 
 
% RA: a 3x3xN matrix with all the rotations matrices 𝑅𝑅𝐴𝐴𝑖𝑖 
% tA: a 3xN matrix with all the translation vectors 𝒕𝒕𝐴𝐴𝑖𝑖 
% RB: a 3x3xN matrix with all the rotations matrices 𝑅𝑅𝐵𝐵𝑖𝑖 
% tB: a 3xN matrix with all the translation vectors 𝒕𝒕𝐵𝐵𝑖𝑖 
% RX: the 3x3 rotation matrix Rx 
% return: the 3x1 translation vector tx 

% create A and b
A = []; b = [];
for i=1:size(RA,3)
    I = eye(3);
    A = [A; (I - RA(:,:,i))];
    b = [b; (tA(:,i) - RX * tB(:,i))];
end

% solve A * tx = b
tx = A \ b;

end