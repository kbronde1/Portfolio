% Assignment 2
% Kiana Bronder (kbronde1)

function X = axxb( e_bh, e_sc ) 
 
% e_bh: a Nx7 matrix that contain N forward kinematics measurements  
% obtained from tf_echo. The format of each row must be 
% [𝑡𝑡𝑥𝑥 𝑡𝑡𝑦𝑦 𝑡𝑡𝑧𝑧 𝑞𝑞𝑥𝑥 𝑞𝑞𝑦𝑦 𝑞𝑞𝑧𝑧 𝑞𝑞𝑤𝑤]  
% e_sc: a Nx7 matrix that contain N AR tag measurements obtained from  
% tf_echo. The format of each row must be [𝑡𝑡𝑥𝑥 𝑡𝑡𝑦𝑦 𝑡𝑡𝑧𝑧 𝑞𝑞𝑥𝑥 𝑞𝑞𝑦𝑦 𝑞𝑞𝑧𝑧 𝑞𝑞𝑤𝑤]  
% return: the 4x4 homogeneous transformation of the hand-eye calibration 
N = size(e_bh, 1);
E = zeros(4, 4, N); S = zeros(4, 4, N);
A = zeros(4, 4, N-1); B = zeros(4, 4, N-1);
RA = zeros(3, 3, N-1); RB = zeros(3, 3, N-1);
tA = []; tB = [];
alphahat = zeros(3, 3, N-1); betahat = zeros(3, 3, N-1);
for i=1:N
    E(:,:,i) = [quat2rotm([e_bh(i,7) e_bh(i,4:6)]) transpose(e_bh(i,1:3)); 0 0 0 1];
    S(:,:,i) = [quat2rotm([e_sc(i,7) e_sc(i,4:6)]) transpose(e_sc(i,1:3)); 0 0 0 1];
    if i>1
        % A1 = E1 \ E2; B1 = S1 / S2;
        A(:,:,i-1) = E(:,:,i-1) \ E(:,:,i); 
        B(:,:,i-1) = S(:,:,i-1) / S(:,:,i);
        RA(:,:,i-1) = A(1:3, 1:3, i-1); RB(:,:,i-1) = B(1:3, 1:3, i-1);
        tA = [tA A(1:3,4,i-1)]; tB = [tB B(1:3,4,i-1)];
        alphahat(:,:,i-1) = logm(RA(:,:,i-1)); % 3x3x(N-1)
        betahat(:,:,i-1) = logm(RB(:,:,i-1));
    end
end
curlyA = curly(alphahat);
curlyB = curly(betahat);
Rx = solveRx(curlyA, curlyB);
tx = solveTx(RA, tA, RB, tB, Rx);
X = [Rx tx; 0 0 0 1];

end

% alphahat: 3x3xN matrix
function A = curly(alphahat)
    % unskew
    A = [];
    for i = 1:size(alphahat,3)
        A = [A unskew(alphahat(:,:,i))];
    end
end

function unskewed = unskew(skewed)
    unskewed = [skewed(3,2); skewed(1,3); skewed(2,1)];
end