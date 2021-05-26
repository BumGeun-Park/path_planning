clear,clc,close all

%% Setting
dt = 0.1;
k = [0.1 0.1];
theta = [0 pi/4 pi/2 3*pi/4 pi 5*pi/4 3*pi/2 7*pi/4];
a = zeros(1,500);
alpha = zeros(1,500);
v = zeros(1,500);
omega = zeros(1,500);
p = zeros(2,500);
video = VideoWriter('trajectory.avi');
open(video);
%% Loop(various theta and iteration)
for n = 1:length(theta)
    initial_configuration = [0 0 theta(n)];
    goal_configuration = [10 10];
    current_configuration = initial_configuration;
    a = norm(goal_configuration-current_configuration(1:2));
    %% iteration (700)
    for t = 1: 700
        a(t) = norm(goal_configuration-current_configuration(1:2));
        LOS = atan((goal_configuration(2)-current_configuration(2))/(goal_configuration(1)-current_configuration(1)));
        alpha(t) = LOS - current_configuration(3);
        v(t) = k(1)*a(t)*cos(alpha(t));
        omega(t) = k(2)*alpha(t) + k(1)*sin(alpha(t))*cos(alpha(t));
        current_configuration(1:2) =  current_configuration(1:2) + [v(t)*cos(current_configuration(3))*dt v(t)*sin(current_configuration(3))*dt];
        current_configuration(3) = current_configuration(3) + omega(t)*dt;
        p(1,t) = current_configuration(1);
        p(2,t) = current_configuration(2);
        %% Video
        if n==5
            figure(6);
            points_x = ...
                [p(1,t)+0.2*sin(current_configuration(3))+0.2*cos(current_configuration(3));
                p(1,t)+0.2*sin(current_configuration(3))-0.2*cos(current_configuration(3));
                p(1,t)-0.2*sin(current_configuration(3))-0.2*cos(current_configuration(3));
                p(1,t)-0.2*sin(current_configuration(3))+0.2*cos(current_configuration(3));
                p(1,t)+0.4*cos(current_configuration(3))];
            points_y = ...
                [p(2,t)-0.2*cos(current_configuration(3))+0.2*sin(current_configuration(3));
                p(2,t)-0.2*cos(current_configuration(3))-0.2*sin(current_configuration(3));
                p(2,t)+0.2*cos(current_configuration(3))-0.2*sin(current_configuration(3));
                p(2,t)+0.2*cos(current_configuration(3))+0.2*sin(current_configuration(3));
                p(2,t)+0.4*sin(current_configuration(3))];
            car = polyshape(points_x,points_y);
            hold off
            plot(car);
            hold on
            plot(p(1,1:t),p(2,1:t),0,0,'*r',10,10,'*m');
            grid on
            xlabel X
            ylabel Y
            legend('Car','trajectory','Starting Point','Goal Point','location','southeast')
            axis([-2 11 -2 11])
            frame = getframe(gcf);
            writeVideo(video,frame);
        end
    end
    %% plot
    figure(1)
    plot(p(1,:),p(2,:));
    hold on
    
    figure(2)
    plot(v);
    hold on
    
    figure(3);
    plot(omega);
    hold on
    
    figure(4)
    plot(a);
    hold on
    
    figure(5);
    plot(alpha);
    hold on
end
%% grid, legend
close(video);
figure(1)
plot(initial_configuration(1),initial_configuration(2),'*r','linewidth',5);
plot(goal_configuration(1),goal_configuration(2),'*m','linewidth',5);
axis([-2 15 -2 15])
grid on
legend('0','(1/4)\pi','(1/2)\pi','(3/4)\pi','\pi','(5/4)\pi','(3/2)\pi','(7/4)\pi','Starting Point','Goal Point','location','southeast')
xlabel ('X [m]')
ylabel ('Y [m]')
title trajectory

figure(2)
grid on
legend('0','(1/4)\pi','(1/2)\pi','(3/4)\pi','\pi','(5/4)\pi','(3/2)\pi','(7/4)\pi')
xlabel Iteration
ylabel ('v [m/s]')

figure(3)
grid on
legend('0','(1/4)\pi','(1/2)\pi','(3/4)\pi','\pi','(5/4)\pi','(3/2)\pi','(7/4)\pi','location','southeast')
xlabel Iteration
ylabel ('\omega [rad/s]')

figure(4)
grid on
legend('0','(1/4)\pi','(1/2)\pi','(3/4)\pi','\pi','(5/4)\pi','(3/2)\pi','(7/4)\pi')
xlabel Iteration
ylabel ('a [m]')

figure(5)
grid on
legend('0','(1/4)\pi','(1/2)\pi','(3/4)\pi','\pi','(5/4)\pi','(3/2)\pi','(7/4)\pi','location','southeast')
xlabel Iteration
ylabel ('\alpha [rad]')