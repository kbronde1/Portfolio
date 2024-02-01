% ASBR Assignment 4
% Kiana Bronder kbronde1

%% calculate g
% given state, v, and w
syms v w dt x y z roll pitch yaw real
s = [x y z roll pitch yaw];
Rx = rotate(roll,'x'); Ry = rotate(pitch,'y'); Rz = rotate(yaw,'z');
R = Rx*Ry*Rz;
t = [x; y; z];
E_rt = [R t; zeros(1,3) 1];
R_rt1 = rotate(w*dt, 'z');
t_rt1 = [v*dt; 0; 0];
E_rt1 = [R_rt1 t_rt1; zeros(1,3) 1];
E = E_rt*E_rt1;
E = simplify(E);
thetax = atan2(E(2,3), E(3,3));
thetay = asin(E(1,3));
thetaz = atan2(E(1,2), E(1,1));
g = [E(1:3,4); thetax; thetay; thetaz];
ccode(g,'file','littleg.txt')
%% calculate G
G = sym('a',6);
% find Jacobian of g
G = simplify(jacobian(g,s));
ccode(G,'file','bigG.txt')

%% calculate WMWt
WMWt = sym('a',6);
% W = Jacobian of process model wrt u
W = sym('a',[6 2]); u = [v;w];
for i = 1:length(g)
    for j = 1:length(u)
        W(i,j) = diff(g(i),u(j));
    end
end
syms a1 a2 a3 a4 real
M = [a1*v^2+a2*w^2 0; 0 a3*v^2+a4*w^2];
WMWt = W * M * W';
ccode(WMWt,'file','WMWt.txt')

%% calculate gps
% each index matches across all vectors
x = [11.965 11.513 9.6168 17.8410533558 3.78440398678 0.927496441638 3.67667756386 9.61683395889...
    0.14188374 -73.3878404097 -1.11294540716 12.3078696503 15.2469424583];
y = [0.245 2.327 4.1078 27.2039455368 19.5518572199 14.3604587625 8.45046579311 4.10777798143...
    0.14265692 118.717275244 -44.4799701499 -27.748554218 -15.4801926525];
z = [-1.461 -1.565 -1.8617 -2.18635230626 -1.44860961853 -1.39443133692 -1.60500746789 -1.86166330955...
    -1.52864754 38.3551689424 2.17165203942 2.67650421728 2.0478137518];
lat = [35.859 35.8595 35.8595 35.8597353162 35.8596348257 35.859582088 35.8595392924 35.8595355812...
    35.859456 35.8603019443 35.8590677617 35.8592458215 35.8593599555];
long = [-108.237 -108.2367 -108.2367 -108.236732019 -108.236858581 -108.236872624 -108.236825823 -108.236780659...
    -108.236838 -108.237982081 -108.236713785 -108.236623122 -108.236629102];
alt = [12.523 12.4128 12.1116 11.8150290596 12.55228056 12.6569134483 12.3555098933 12.187974594...
    12.4886104 52.3418374758 16.1517299818 16.6694914545 16.0359220302];

zToAlt = polyfit(z,alt,1);
yToLong = polyfit(y,long,1);
xToLat = polyfit(x,lat,1);
fprintf('x: %f %f\ny: %f %f\nz: %f %f\n', xToLat, yToLong, zToAlt)

%% calculate R
covX = cov(x,lat); covY = cov(y,long); covZ = cov(z,alt);
fprintf('Expected x error = %f\n', covX(2,1))
fprintf('Expected y error = %f\n', covY(2,1))
fprintf('Expected z error = %f\n', covZ(2,1))

q = readmatrix('imu.txt'); % each row is a quat
qnew = [];
for i = 1:100:size(q,1)
    qnew = [qnew; q(i,:)];
end
[roll, pitch, yaw] = quat2angle(qnew(:,2:5));
RPY_imu = [roll pitch yaw];

pose = readmatrix('pose.txt');
posenew = []; RPY = [];
for i = 1:size(qnew,1)
    idx = ceil(i * 26140/5911);
    [R, P, Y] = quat2angle(pose(idx,5:8));
    RPY = [RPY ; R P Y];
end

Rerror = cov(RPY(:,1), RPY_imu(:,1));
Perror = cov(RPY(:,2), RPY_imu(:,2));
Yerror = cov(RPY(:,3), RPY_imu(:,3));

fprintf('Expected R error = %f\n', Rerror(2,1))
fprintf('Expected P error = %f\n', Perror(2,1))
fprintf('Expected Y error = %f\n', Yerror(2,1))
%% calculate Hgps
Hgps = sym('a',3);
syms h1 h2 h3 x y z real;
h = [h1*x; h2*y; h3*z];
Hgps = simplify(jacobian(h,[x y z]));
ccode(Hgps,'file','Hgps.txt')

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