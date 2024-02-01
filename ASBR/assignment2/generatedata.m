% Assignment 2
% Kiana Bronder (kbronde1)


% Generate synthetic data to test hand-eye calibration. 
% e_bh and e_sc are Nx7 matrices that represent N E_bh and N E_sc  
% transformations. Each row is of the form [ tx ty tz qx qy qz qw ] 
% were tx, ty and tz denote a translation and qx, qy, qz, qw a  
% quaternion. 
% X is a randomly generated hand-eye transformation 

function [e_bh, e_sc, X] = generatedata(N)

% Assume we know the transformation between the world and the  
% checkerboard 
E_bc = [ eye(3) [ 1; 0; 0 ]; 0 0 0 1 ]; 
 
% Create a random X for generating the data 
X = randSE3();
e_bh = []; 
e_sc = []; 

for i=1:N 
  % Now that you have X and E_bc 
% TODO: Generate a random E_bh, you can use rotm2quat to convert  
%       the rotation matrix to a quaternion (be careful because it  
%    outputs in the format [qw, qx, qy, qz]) and append the  
%    transformation to e_bh 
    E_bh = randSE3();
    quat = rotm2quat(E_bh(1:3,1:3));
    append = [E_bh(1,4) E_bh(2,4) E_bh(3,4) quat(2) quat(3) quat(4) quat(1)];
    e_bh = [e_bh; append];

% Now that you have X, E_bc and E_bh 
% TODO: Find E_sc and append the transformation to e_sc 
    E_sc = inv(X) * inv(E_bh) * E_bc;
    quat = rotm2quat(E_sc(1:3,1:3));
    append = [E_sc(1,4) E_sc(2,4) E_sc(3,4) quat(2) quat(3) quat(4) quat(1)];
    e_sc = [e_sc; append];
end

end
 
% Generate a random SE3 transformation 
function Rt = randSE3() 
    % TODO: Generate a random rotation matrix 
    vec = rand(1,4);
    norm_vec = vec / norm(vec);
    R = quat2rotm(norm_vec);
    % TODO: Generate a random translation 
    t =  randn(3,1);
    Rt = [R t; 0 0 0 1]; 
%     identity = R*transpose(R)
end