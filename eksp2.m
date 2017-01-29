%% 2
%
% PL: Wyznaczenie hipotetycznego modelu matematycznego 
% mechaniki robota w celu wyznaczenia częstotliwości drgań własnych układu 
% oraz symulacja wartości parametrów kinematycznych 
% na wymuszenie modelujące przejazd.
%
% ENG: Mathematical modeling mechanical system of robot to find natural
% freaquencies of system. Analysis of kinematic parameters in time and frequency domain i.e.
% displacement, velocity and acceleration as a response for input signal -
% simulation of ride.
%
%% PARAMETERS DEF
%
clear all
clc

init = num2cell([0.2 0.1 0.05 0.05]);  % masses of bodies
[m1, m2, m3, m4] = deal(init{:});

init = num2cell([2 12 10 15]);  % elasticity coefs
[k1, k2, k3, k4] = deal(init{:});

init = num2cell([0.3 0.08 0.09 0.1]);  % damping coefs
[c1, c2, c3, c4] = deal(init{:});

A = [
0,1,0,0,0,0,0,0;
-(k1+k2),-(c1+c2),(k2),(c2),(0),(0),(0),(0);
0,0,0,1,0,0,0,0;
(k2),(c2),-(k2+k3+k4),-(c2+c3+c4),(k3),(c3),(k4),(c4);
0,0,0,0,0,1,0,0;
(0),(0),(k3),(c3),(-k3),(-c3),(0),(0);
0,0,0,0,0,0,0,1;
(0),(0),(k4),(c4),(-k4),(-c4),(0),(0);
]

B = [
  0,0,0,0;
  1/m1,0,0,0;
  0,0,0,0;
  0,1/m2,0,0;
  0,0,0,0;
  0,0,1/m3,0;
  0,0,0,0;
  0,0,0,1/m4;
]

C = [
    1,0,0,0,0,0,0,0;
    0,0,1,0,0,0,0,0;
    0,0,0,0,1,0,0,0;
    0,0,0,0,0,0,1,0; 

];

D = [0];

sys = ss(A,B,C,D);
[wn,z] = damp(sys);
wn = wn(1:2:length(wn))/2/pi %  [Hz]
z = z(1:2:length(z))*100    %   [%]



t = [0 : 0.01 : 20];
U = [[1; zeros((length(t)-1),1)],zeros((length(t)),1),zeros((length(t)),1),zeros((length(t)),1)];
u_noise = [  wgn(length(t), 1, 0), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1) ]
Y = lsim(sys,u_noise,t);

figure
plot(t,Y(:,1))
figure
plot(t,Y(:,2))
figure
plot(t,Y(:,3))
figure
plot(t,Y(:,4))






