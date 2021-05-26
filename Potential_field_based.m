clear,clc,close all
%% Set up
ka = [0.01 0.1];
kr = [0.01 0.1];
r0 = 3;
Iteration = 1000;
Initial_location = [0 0];
Target_location = [10 8];
obstacle1 = [5 4.3];
obstacle2 = [7.5 5];
obstacle3 = [3 2.5];
current_location = Initial_location;

%% Calculate formular
trace = zeros(Iteration,2);
dist_goal = zeros(Iteration,1);
dist_obstacle1 = zeros(Iteration,1);
dist_obstacle2 = zeros(Iteration,1);
dist_obstacle3 = zeros(Iteration,1);
for i = 1:2
    for j = 1:2
        current_location = [0 0];
        for t = 1:Iteration
            trace(t,1:2) = current_location;
            dist_goal(t) = norm(Target_location-current_location);
            dist_obstacle1(t) =  norm(obstacle1 - current_location);
            dist_obstacle2(t) =  norm(obstacle2 - current_location);
            dist_obstacle3(t) =  norm(obstacle3 - current_location);
            dUa = ka(j)*(current_location - Target_location);
            
            % obstacle1
            if (norm(obstacle1 - current_location)<=r0)
                dUr1 = kr(i)*(1/r0 - 1/norm(obstacle1-current_location))*(current_location-obstacle1)/norm(obstacle1-current_location)^3;
            else
                dUr1 = [0 0];
            end
            
            % obstacle2
            if (norm(obstacle2 - current_location)<=r0)
                dUr2 = kr(i)*(1/r0 - 1/norm(obstacle2-current_location))*(current_location-obstacle2)/norm(obstacle2-current_location)^3;
            else
                dUr2 = [0 0];
            end
            % obstacle3
            if (norm(obstacle3 - current_location)<=r0)
                dUr3 = kr(i)*(1/r0 - 1/norm(obstacle3-current_location))*(current_location-obstacle3)/norm(obstacle3-current_location)^3;
            else
                dUr3 = [0 0];
            end
            % update
            current_location = current_location - (dUa+dUr1+dUr2+dUr3);
        end
        
        %% show trace
        figure(1)
        hold on
        plot(trace(:,1),trace(:,2));
        hold off
        
        figure(2)
        hold on
        plot(1:Iteration,dist_goal);
        hold off
        
        figure(3)
        hold on
        plot(1:Iteration,dist_obstacle1);
        hold off
        
        figure(4)
        hold on
        plot(1:Iteration,dist_obstacle2);
        hold off
        
        figure(5)
        hold on
        plot(1:Iteration,dist_obstacle3);
        hold off
    end
end
figure(1)
hold on
plot(obstacle1(1),obstacle1(2),'r*',obstacle2(1),obstacle2(2),'r*',obstacle3(1),obstacle3(2),'r*',Initial_location(1),Initial_location(2),'g*',Target_location(1),Target_location(2),'m*');
legend('ka = 0.01,kr = 0.01','ka = 0.1,kr = 0.01','ka = 0.01,kr = 0.1','ka = 0.1,kr = 0.1','Obstacle1','Obstacle2','Obstacle3','Start','Target','location','northwest')
grid on
xlabel X
ylabel Y
title Trace
hold off

figure(2)
hold on
legend('ka = 0.01,kr = 0.01','ka = 0.1,kr = 0.01','ka = 0.01,kr = 0.1','ka = 0.1,kr = 0.1')
grid on
xlabel Iteration
ylabel distance
title('Distance to the Final Position')
hold off

figure(3)
hold on
plot(1:Iteration,r0*ones(1,Iteration),'.-r');
legend('ka = 0.01,kr = 0.01','ka = 0.1,kr = 0.01','ka = 0.01,kr = 0.1','ka = 0.1,kr = 0.1','r0 = 3','location','southeast')
grid on
xlabel Iteration
ylabel distance
title('Distance to the Obstacle1')
hold off

figure(4)
hold on
plot(1:Iteration,r0*ones(1,Iteration),'.-r');
legend('ka = 0.01,kr = 0.01','ka = 0.1,kr = 0.01','ka = 0.01,kr = 0.1','ka = 0.1,kr = 0.1','r0 = 3','location','southeast')
grid on
xlabel Iteration
ylabel distance
title('Distance to the Obstacle2')
hold off

figure(5)
hold on
plot(1:Iteration,r0*ones(1,Iteration),'.-r');
legend('ka = 0.01,kr = 0.01','ka = 0.1,kr = 0.01','ka = 0.01,kr = 0.1','ka = 0.1,kr = 0.1','r0 = 3','location','southeast')
grid on
xlabel Iteration
ylabel distance
title('Distance to the Obstacle3')
hold off