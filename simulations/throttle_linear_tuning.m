clc; clear all; close all;

% simulation to figure out linear thottle tuning with specific values
figure(1);

min_step_time = 500;
max_step_time = 1000000;

max_throttle = 255;

t = 0:max_throttle-1;

map_t = t .* ( ((max_step_time/min_step_time)-1)/(max_throttle-1) ) + 1;

% show step time to see if it starts at step_time and ends at min_step_time
step_time = max_step_time ./ map_t

hold on;
y = 1./(max_step_time./t);
plot(t,y, 'b','LineWidth',2);
y1 = 1./(max_step_time./t - (max_step_time/max_throttle) + min_step_time);
plot(t,y1, 'r','LineWidth',2);
y2 = 1./step_time;
plot(t,y2, 'g','LineWidth',2);
xlabel("Throtle");
ylabel("Speed");
legend("Should be linear like this", "first attempt", "last attempt")

figure(2);
plot(t,step_time, 'r','LineWidth',3);
xlabel("Throttle");
ylabel("Step time");