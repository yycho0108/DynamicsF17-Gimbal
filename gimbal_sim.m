%function [t,y] = gimbal_sim()
%% Note:
% if mtimesx complains about "A C/C++ compiler has not been selected with mex -setup", then
% run:
% mex -setup C++

%% Imports ...
% for using mtimesx.
addpath('./libs');
%% define parameters ...
m=0.4; %kg
lx=0.2; %m
ly=0.1; %m
lz=0.08; %m
g=9.81; %m/s
K = [3.16228 0 4.05293 0 1.01608 0;
    0 3.16228 0 4.04843 0 1.01031]; % gain matrix
params = {m, lx, ly, lz, g, K};

%% ODE ...
odefun = @(t,x) delta(t,x,params);
tspan = [0 10];
tx = deg2rad(20);
ty = deg2rad(30);
y0 = [0 0 tx ty 0 0]';
[t,y] = ode45(odefun, tspan, y0);

%% plot time vs. angles
figure;
hold on;

tx = y(:,3);
ty = y(:,4);
tz = ty*0;

plot(t, tx);
plot(t, ty);
legend('\theta_x','\theta_y');

%% plot payload position
figure;
px = lx+lz*sin(y(:,4));
py = px*0;
pz = -lz*cos(y(:,4)) + ly*sin(y(:,3));
p = [px py pz];
plot3(px,px*0,pz);

%% plot as cup ...
in2m = 0.0254;
r_b = 2.32/2 * in2m; %m
r_t = 3.5/2 * in2m; %m
r=linspace(r_b,r_t,10);
h = 0.11; %m
h_cm = 1.57 * in2m; %height of attachment

figure;
[cx, cy, cz] = cylinder(r);

cx = reshape(cx, [], 1);
cy = reshape(cy, [], 1);
cz = reshape(cz, [], 1);
cz = cz * h - h_cm;
cup = [cx cy cz]';

% rotate ...
rotm = eul2rotm([tx ty 0*tx]);
t_cup = mtimesx(rotm, cup);

% translate ...
pts = t_cup + reshape(p', 3, 1, []);

tri = delaunay(cx,cy);
n = length(tx);

%hCup = trisurf(tri,cx,cy,cz);
% rotm(:,:,1) * [cx, cy, cz]'

%% Run Animation
n = 800;

% limits ...
mxs = max(max(pts, [],2),[],3);
mns = min(min(pts, [],2),[],3);

figure;

cx = pts(1,:,1);
cy = pts(2,:,1);
cz = pts(3,:,1);
h = trisurf(tri,cx,cy,cz);
h.EdgeColor = 'none';

for i=2:n
    h.Vertices = squeeze(pts(:,:,i))';
    xlim([mns(1) mxs(1)]);
    ylim([mns(2) mxs(2)]);
    zlim([mns(3) mxs(3)]);
    drawnow;
end
%% odefun
function dx=delta(~, x, params)
% unroll parameters
[m, ~, ly, lz, g, K] = deal(params{:});

% unroll state
c_x = num2cell(x);
[~, ~, tx, ty, wx, wy] = deal(c_x{:});

% control input, torque
u = -K*x;
ux = u(1);
uy = u(2);

alpha = [(1/8).*ly.^(-2).*m.^(-1).*cos(tx).^2.*(lz.*cos(ty)+(-1).*ly.*sin( ...
    tx)).^(-2).*((-8).*ly.*lz.*uy.*sec(tx).^4.*sin(ty)+ux.*sec(tx).*( ...
    8.*lz.^2.*cos(ty).^2.*sec(tx)+(ly.^2+4.*lz.^2+(-1).*ly.^2.*cos(4.* ...
    tx)+(-4).*lz.^2.*cos(2.*ty)).*sec(tx).^3+(-16).*ly.*lz.*cos(ty).* ...
    tan(tx))+(-1).*m.*(4.*ly.*lz.*wx.*wy.*sec(tx).^3.*sin(ty).*(3.* ...
    ly.*lz.*cos(2.*tx+(-1).*ty)+(-2).*ly.*lz.*cos(ty)+3.*ly.*lz.*cos( ...
    2.*tx+ty)+2.*ly.^2.*sin(tx)+2.*lz.^2.*sin(tx)+(-2).*ly.^2.*sin(3.* ...
    tx)+lz.^2.*sin(tx+(-2).*ty)+lz.^2.*sin(tx+2.*ty))+2.*g.*ly.*(4.* ...
    lz.^2.*cos(ty).^2.*sec(tx)+(3.*ly.^2+(-4).*lz.^2+6.*ly.^2.*cos(tx) ...
    +4.*ly.^2.*cos(2.*tx)+2.*ly.^2.*cos(3.*tx)+ly.^2.*cos(4.*tx)+4.* ...
    lz.^2.*cos(2.*ty)).*sec(tx).^4.*sin((1/2).*tx).^2+(-8).*ly.*lz.* ...
    cos(ty).*tan(tx))+(-1).*wy.^2.*sec(tx).*((-8).*lz.^4.*cos(ty).^4.* ...
    sin(tx)+(-8).*ly.*lz.^3.*cos((1/2).*tx).^2.*cos(ty).*sec(tx).*((7+ ...
    (-10).*cos(tx)+5.*cos(2.*tx)).*cos(ty).^2+2.*(3+(-2).*sec(tx)).* ...
    sin(ty).^2)+lz.^2.*cos(ty).^2.*(3.*ly.^2+(-4).*lz.^2+16.*ly.^2.* ...
    cos(tx)+12.*ly.^2.*cos(2.*tx)+9.*ly.^2.*cos(4.*tx)+4.*lz.^2.*cos( ...
    2.*ty)).*sec(tx).*tan(tx)+(-2).*ly.^3.*lz.*(4+5.*cos(tx)+7.*cos( ...
    3.*tx)).*cos(ty).*sin(tx).*tan(tx)+8.*ly.^2.*(ly.^2.*cos(2.*tx).* ...
    sin(tx).^3+lz.^2.*(1+cos(2.*tx).*sec(tx)).*sin(ty).^2.*tan(tx))))) ...
    ,ly.^(-1).*m.^(-1).*(lz.*cos(ty)+(-1).*ly.*sin(tx)).^(-2).*(ly.* ...
    uy.*sec(tx).^2+(-1).*lz.*ux.*sec(tx).^2.*sin(ty)+m.*(ly.*wx.*wy.*( ...
    2.*ly.*lz.*cos(tx).*cos(ty)+(-1).*ly.^2.*sin(2.*tx)+2.*(lz.*cos( ...
    ty)+(-1).*ly.*sin(tx)).^2.*tan(tx))+lz.*((-1).*g.*ly.*((-1)+sec( ...
    tx)).*sec(tx).*sin(ty)+wy.^2.*(ly.*lz.*cos((1/2).*tx).^2.*sin(2.* ...
    ty)+lz.^2.*cos(ty).^2.*sin(ty).*tan(tx)+ly.*sin(tx).*((-1).*lz.* ...
    sin(2.*ty).*tan(tx)+(-1).*ly.*sin(ty).*(1+cos(tx)+(-1).*sin(tx).* ...
    tan(tx)))))))];

% angular acc.
ax = alpha(1);
ay = alpha(2);

dx = [tx, ty, wx, wy, ax, ay]';
end