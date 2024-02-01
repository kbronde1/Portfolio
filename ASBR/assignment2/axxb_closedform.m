% Assignment 2
% Kiana Bronder (kbronde1)s

function X = axxb_closedform(e_bh, e_sc) 


% function description
% e_bh: a 3x7 matrix that contain 3 FK measurements obtained from tf_echo.
% The format of each row must be [𝑡𝑡𝑥𝑥 𝑡𝑡𝑦𝑦 𝑡𝑡𝑧𝑧 𝑞𝑞𝑥𝑥 𝑞𝑞𝑦𝑦 𝑞𝑞𝑧𝑧 𝑞𝑞𝑤𝑤]  
% e_sc: a 3x7 matrix that contain 3 AR tag measurements obtained from  
% tf_echo. The format of each row must be [𝑡𝑡𝑥𝑥 𝑡𝑡𝑦𝑦 𝑡𝑡𝑧𝑧 𝑞𝑞𝑥𝑥 𝑞𝑞𝑦𝑦 𝑞𝑞𝑧𝑧 𝑞𝑞𝑤𝑤]  
% return: the 4x4 homogeneous transformation of the hand-eye calibration

% build E and S
E1 = [quat2rotm([e_bh(1,7) e_bh(1,4:6)]) transpose(e_bh(1,1:3)); 0 0 0 1];
E2 = [quat2rotm([e_bh(2,7) e_bh(2,4:6)]) transpose(e_bh(2,1:3)); 0 0 0 1];
E3 = [quat2rotm([e_bh(3,7) e_bh(3,4:6)]) transpose(e_bh(3,1:3)); 0 0 0 1];
S1 = [quat2rotm([e_sc(1,7) e_sc(1,4:6)]) transpose(e_sc(1,1:3)); 0 0 0 1];
S2 = [quat2rotm([e_sc(2,7) e_sc(2,4:6)]) transpose(e_sc(2,1:3)); 0 0 0 1];
S3 = [quat2rotm([e_sc(3,7) e_sc(3,4:6)]) transpose(e_sc(3,1:3)); 0 0 0 1];

% calculate A and B
A1 = E1 \ E2; A2 = E2 \ E3;
B1 = S1 / S2; B2 = S2 / S3;
% A1 * xs - xs * B1

% calculate alpha and beta
RA1 = A1(1:3,1:3); RB1 = B1(1:3,1:3);
a1 = logm(RA1); b1 = logm(RB1);
alpha1 = [a1(3,2); a1(1,3); a1(2,1)]; % unskew
beta1 = [b1(3,2); b1(1,3); b1(2,1)];
RA2 = A2(1:3,1:3); RB2 = B2(1:3,1:3);
a2 = logm(RA2); b2 = logm(RB2);
alpha2 = [a2(3,2); a2(1,3); a2(2,1)];
beta2 = [b2(3,2); b2(1,3); b2(2,1)];
alpha3 = cross(alpha1, alpha2);
beta3 = cross(beta1, beta2);

% build curly A and B
A = [alpha1 alpha2 alpha3];
B = [beta1 beta2 beta3];

% build X
Rx = A / B;
RAminusI = [(RA1 - eye(3)); (RA2 - eye(3))];
RxtBminustA = [(Rx * B1(1:3,4) - A1(1:3,4)); (Rx * B2(1:3,4) - A2(1:3,4))];
tx = RAminusI \ RxtBminustA;
X = [Rx tx; 0 0 0 1];

end