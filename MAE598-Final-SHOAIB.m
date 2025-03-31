clearvars; clc; close all
a = 1; b = 1;
p1 = 1; p2 = -0.2172;

B = [0;1];
C = [1 0];
%% random ps
p=zeros(20,1);
p=(p1-p2).*rand(20,1)+p2;
for i=1:20
    A(:,:,i) = [0 1;-b*p(i) -a];
end
%%
A(:,:,1) = [0 1;-b*p1 -a];
A(:,:,2) = [0 1;-b*p2 -a];
np = size(A,3);

nx = size(A,1);
nu = size(B,2);
ny = size(C,1);

nw = nu + ny;
nz = ny + nu;

B1 = [zeros(nx,ny) B];
B2 = B;
C1 = [zeros(nu,nx); C];
D11 = [zeros(ny,nw); eye(ny) zeros(ny,nu)];
D12 = [eye(nu); zeros(ny,nu)];

C2 = C;
D21 = [eye(ny) zeros(ny,nu)];
D22 = zeros(ny,nu);

P = ss(A,[B1 B2],[C1;C2],[D11,D12;D21,D22]);
P.InputGroup.w  = 1:nw;
P.InputGroup.u  = (nw+1):(nw+nu);
P.OutputGroup.z = 1:nz;
P.OutputGroup.y = (nz+1):(nz+ny);
%%
% NR = [   eye(nx),    zeros(nx,ny);...
%       zeros(ny,nx),    eye(ny);...
%           -B',       zeros(nu,ny)];  
NR = null([B2',D12'],'r');
% NS = [   eye(nx),    zeros(nx,nu);...
%       zeros(nu,nx),    eye(nu);...
%           -C,        zeros(ny,nu)];
NS = null([C2,D21],'r');
      
NNR = [NR,zeros(nx+nz,nw);zeros(nw,size(NR,2)),eye(nw)];
NNS = [NS,zeros(nx+nw,nz);zeros(nz,size(NS,2)),eye(nz)];

R = sdpvar(nx,nx);
S = sdpvar(nx,nx);
g = sdpvar(1,1);
eps = 1e-2;
%% LMIs 
LMI0 = [R,eye(nx);eye(nx), S] >= eps*eye(2*nx);
for k=1:20
LMI1{k} = [A(:,:,k)*R + R*A(:,:,k)' - g*B*B',     R*C',       zeros(nx,ny),        B;...
                        C*R,                  -g*eye(ny),        eye(ny),    zeros(ny,nu);...
                    zeros(ny,nx),                eye(ny),      -g*eye(ny),   zeros(ny,nu);
                         B',                  zeros(nu,ny),   zeros(nu,ny),   -g*eye(nu)] <= -eps*eye(nx+ny+ny+nu);       

LMI2{k} = [A(:,:,k)'*S + S*A(:,:,k) - g*C'*C,     S*B,       zeros(nx,ny),    zeros(nx,nu);...
                        B'*S,                  -g*eye(ny),   zeros(ny,ny),    zeros(ny,nu);...
                    zeros(ny,nx),             zeros(ny,ny),      -g*eye(ny),   zeros(ny,nu);
                    zeros(nu,nx),             zeros(nu,ny),   zeros(nu,ny),   -g*eye(nu)] <= -eps*eye(nx+ny+ny+nu);       
end
LMIs = [LMI0,LMI1{1},LMI1{2},LMI2{1},LMI2{2}];
%%
%options = sdpsettings('solver','mosek','verbose',0);
optimize(LMIs,g)
g = value(g)
%%
MN = eye(2) - value(R)*value(S);
[U,SS,V] = svd(MN);
M = U*SS; N = V;
PI1 = [eye(nx), value(R); zeros(nx,nx), M'];
PI2 = [value(S), eye(nx); N', zeros(nx,nx)];
Xcl = PI2/PI1;
eig(Xcl)
%% Controller
for k=1:np
Ak = sdpvar(nx,nx);
Bk = sdpvar(nx,ny);
Ck = sdpvar(nu,nx);
Dk = sdpvar(nu,ny);

Acl = [A(:,:,k) + B2*Dk*C2, B2*Ck; Bk*C2, Ak];
Bcl = [B1 + B2*Dk*D21; Bk*D21];
Ccl = [C1 + D12*Dk*C2, D12*Ck];
Dcl = D11 + D12*Dk*D21;

eps = 1e-8;
LMI = [Acl'*Xcl + Xcl*Acl,   Xcl*Bcl,      Ccl';...
          Bcl'*Xcl,        -g*eye(nw),     Dcl';...
            Ccl,              Dcl,     -g*eye(nz)] <= -eps*eye(nx+nx+nw+nz);

optimize(LMI,[])
K(:,:,k) = ss(value(Ak),value(Bk),value(Ck),value(Dk));
end
%%
for k=1:np
Gcl(:,:,k) = lft(P(:,:,k),K(:,:,k));
end
fprintf('GAMMA: %g\t MATLAB: [%g,%g]\n',g,norm(Gcl,inf));
fprintf('Max. real(eig(Gcl)): %g\n', max(max(real(eig(Gcl)))));
t=0:1:30; 
ii = (1:20)';
step(Gcl,t)