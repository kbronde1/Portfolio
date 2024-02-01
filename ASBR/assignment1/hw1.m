% Assignment 1
% Kiana Bronder (kbronde1)
%% part 4
% RGB = XYZ
world_to_base = [0; 0; 0];
base_to_shoulder = [0; 0; .089159];
shoulder_to_upperarm = [0; 0; 0];
upperarm_to_forearm = [-0.425; 0; 0];
forearm_to_wrist1 = [-0.39225; 0; 0.10915];
wrist1_to_wrist2 = [0; -0.09465; 0];
wrist2_to_wrist3 = [0; 0.0823; 0];
wrist3_to_gripper = [0; 0; 0.13931]; % constant

% q1 = 0; % shoulder pan
% q2 = -1.56; % shoulder lift
% q3 = -0.49; % elbow
% q4 = -0.08; % wrist 1
% q5 = 0.27; % wrist 2
% q6 = 5.04; % wrist 3
syms q1 q2 q3 q4 q5 q6;

R0 = rotate(sym(pi), 'z');
R1 = rotate(q1, 'z');
R2 = rotate(q2, 'z');
R22 = rotate(sym(pi)/2, 'x');
R3 = rotate(q3, 'z');
R4 = rotate(q4, 'z');
R5 = rotate(q5, 'z');
R6 = rotate(q6, 'z');
R66 = rotate(-sym(pi)/2, 'x');
R7 = rotate(-sym(pi)/2, 'z');

% base
world = eye(4); % [0; 0; 0; 1]
w_E_b = frame(R0, world_to_base);
% base = world * w_E_b;
% fprintf("The base w.r.t. world: [%f %f %f]\n", base(1:3, 4));
% Translation: [0 0 0] :)
% Quat: [0 0 1 0] :)

% shoulder
b_E_s = frame(R1, base_to_shoulder);
% shoulder = base * b_E_s;
% fprintf("The shoulder w.r.t. world: [%f %f %f]\n", shoulder(1:3, 4));
% Translation: [0.000, 0.000, 0.089] :)
% Quat: [0 0 0 1] :)

% upper arm
s_E_u = frame(R22*R2, shoulder_to_upperarm);
% upperarm = shoulder * s_E_u;
% fprintf("The upper arm w.r.t. world: [%f %f %f]\n", upperarm(1:3, 4));
% Translation: [0, 0, 0.089] :)
% Quat: [.503 .497 -.497 .503] :)

% forearm
u_E_f = frame(R3, upperarm_to_forearm);
% forearm = upperarm * u_E_f;
% fprintf("The forearm w.r.t. world: [%f %f %f]\n", forearm(1:3, 4));
% Translation: [0 0 0.514] :)
% (Rel) Quat: [0 0 -0.243 0.969] :)

% wrist 1
f_E_w1 = frame(R4, forearm_to_wrist1);
% wrist1 = forearm * f_E_w1;
% fprintf("Wrist 1 w.r.t. world: [%f %f %f]\n", wrist1(1:3, 4));
% Translation: [0.177, -0.109, 0.861] :(
% (Rel) Quat: [0 0 -0.039 0.999] :)

% wrist 2
w1_E_w2 = frame(R22*R5, wrist1_to_wrist2);
% wrist2 = wrist1 * w1_E_w2;
% fprintf("Wrist 2 w.r.t. world: [%f %f %f]\n", wrist2(1:3, 4));
% Translation: [0.097 -0.109 0.912] 
% Rel Quat: [0.700 -0.095 0.095 0.700] :)

% wrist 3
w2_E_w3 = frame(R66*R6, wrist2_to_wrist3);
% wrist3 = wrist2 * w2_E_w3;
% fprintf("Wrist 3 w.r.t. world: [%f %f %f]\n", wrist3(1:3, 4));
% Translation: [0.109 -0.188 0.930]
% Rel Quat: [-0.574 -0.411 -0.411 0.574] :)

% gripper
w3_E_g = frame(R7, wrist3_to_gripper);
% gripper = wrist3 * w3_E_g;
% fprintf("The gripper w.r.t. world: [%f %f %f]\n", gripper(1:3, 4));
% Translation: [0.128 -0.322 0.963]
% Rel Quat: [0 0 0 1] (w.r.t. right inner finger pad??)

%% part 5

q = [q1 q2 q3 q4 q5 q6];
E = w_E_b * b_E_s * s_E_u * u_E_f * f_E_w1 * w1_E_w2 * w2_E_w3 * w3_E_g;
J = jacobian(E, q);

syms e00 e01 e02 e03;
syms e10 e11 e12 e13;
syms e20 e21 e22 e23;
E = [e00 e01 e02 e03; e10 e11 e12 e13; e20 e21 e22 e23; 0 0 0 1];
R = E(1:3,1:3);
t = E(1:3,4);
that = [0 -t(3) t(2); t(3) 0 -t(1); -t(2) t(1) 0];
A = [R that*R; zeros(3) R];

% convert to C code
% ccode(J,'file','J.txt')
% ccode(E,'file','E.txt')
% ccode(inv(E),'file','Einv.txt')
% ccode(inv(A),'file','Ainv.txt')

%% functions
function R = rotate(q, axis)  
    if axis == 'x'
            R = [1 0 0; 0 cos(q) -sin(q); 0 sin(q) cos(q)];
        elseif axis == 'y'
            R = [cos(q) 0 sin(q); 0 1 0; -sin(q) 0 cos(q)];
        else
            R = [cos(q) -sin(q) 0; sin(q) cos(q) 0; 0 0 1];
    end
end

function E = frame(R, t)
    E = [R t; 0 0 0 1];
end

function J = jacobian(E, q)
    for i = 1:length(q)
        curr = diff(E, q(i)) / E; % multiply by IFK
        tempv = curr(1:3, 4); % find v components
        tempw = invskew(curr); % find w components
        if i == 1 % build v and w
            v = tempv;
            w = tempw;
        else
            v = [v tempv];
            w = [w tempw];
        end
    end
    J = [v; w]; % build Jacobian
    J = simplify(J);
end

function w = invskew(E)
    w_x = -E(2,3);
    w_y = E(1,3);
    w_z = E(2,1);
    w = [w_x; w_y; w_z];
end