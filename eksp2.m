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

init = num2cell([0.1 0.15 0.05 0.02]);  % masses of bodies
[m1, m2, m3, m4] = deal(init{:});


init = num2cell([1000 1000 6000 1500]);  % elasticity coefs
[k1, k2, k3, k4] = deal(init{:});

init = num2cell([7.9 12.6 12.2 9.3]);  % damping coefs
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

%% Calculations
%
sys = ss(A,B,C,D);
[wn,z] = damp(sys);
wn = wn(1:2:length(wn))/2/pi %  [Hz]
z = z(1:2:length(z))*100    %   [%]



t = [0 : 0.01 : 4];
U = [[1; zeros((length(t)-1),1)],zeros((length(t)),1),zeros((length(t)),1),zeros((length(t)),1)];
Y = lsim(sys,U,t);
%% Impulse on 1. mass - track defect
%

figure
subplot(2,2,1)
plot(t,Y(:,1))
title('1. mass response')
subplot(2,2,2)
plot(t,Y(:,2))
title('2. mass response')
subplot(2,2,3)
plot(t,Y(:,3))
title('3. mass response')
subplot(2,2,4)
plot(t,Y(:,4))
title('4. mass response')
%% Different wheel elasticity
%
init = num2cell([5000 1000 6000 1500]);  % elasticity coefs
[k1, k2, k3, k4] = deal(init{:});

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

sys = ss(A,B,C,D);
Y = lsim(sys,U,t);

figure
subplot(2,2,1)
plot(t,Y(:,1))
title('1. mass response')
subplot(2,2,2)
plot(t,Y(:,2))
title('2. mass response')
subplot(2,2,3)
plot(t,Y(:,3))
title('3. mass response')
subplot(2,2,4)
plot(t,Y(:,4))
title('4. mass response')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
init = num2cell([25000 1000 6000 1500]);  % elasticity coefs
[k1, k2, k3, k4] = deal(init{:});

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

sys = ss(A,B,C,D);
Y = lsim(sys,U,t);

figure
subplot(2,2,1)
plot(t,Y(:,1))
title('1. mass response')
subplot(2,2,2)
plot(t,Y(:,2))
title('2. mass response')
subplot(2,2,3)
plot(t,Y(:,3))
title('3. mass response')
subplot(2,2,4)
plot(t,Y(:,4))
title('4. mass response')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% simulate other responses and plot results.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% driving atlitude

t = [0.01 : 0.005 : 4];
U = [sin(0.15*pi*t + pi/2)', zeros((length(t)),1),zeros((length(t)),1),zeros((length(t)),1)];
% u_noise = [  wgn(length(t), 1, 0), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1) ]
Y = lsim(sys,U,t);

figure
plot([1:length(t)], sin(0.15*pi*t + pi/2))
title('Example of force input on 1. mass element')

figure
subplot(2,2,1)
plot(t,Y(:,1))
title('1. mass response')
subplot(2,2,2)
plot(t,Y(:,2))
title('2. mass response')
subplot(2,2,3)
plot(t,Y(:,3))
title('3. mass response')
subplot(2,2,4)
plot(t,Y(:,4))
title('4. mass response')




%% change mases and or coefs -> calc Wn Z and responses.
% The idea behind natural frequencies, demonstration
%
t = 0 :0.005: 10;
u_res = [[sin(2*pi*wn(1)*t)]', [sin(2*pi*wn(2)*t)]', [sin(2*pi*wn(3)*t)]', [sin(2*pi*wn(4)*t)]' ]
Y_res = lsim(sys,u_res,t);

figure
subplot(2,2,1)
plot(t,Y_res(:,1))
title('1. mass response')
xlabel('Time')
ylabel('Magnitude of displacement')

subplot(2,2,2)
plot(t,Y_res(:,2))
title('2. mass response')
xlabel('Time')  
ylabel('Magnitude of displacement')

subplot(2,2,3)
plot(t,Y_res(:,3))
title('3. mass response')
xlabel('Time')
ylabel('Magnitude of displacement')

subplot(2,2,4)
plot(t,Y_res(:,4))
title('4. mass response')
xlabel('Time')
ylabel('Magnitude of displacement')

