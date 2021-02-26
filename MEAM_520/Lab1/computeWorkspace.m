x = zeros(25000,1);
y = zeros(25000,1);
z = zeros(25000,1);

theta1min = -1.4;
theta1max = 1.4;
theta2min = -1.2;
theta2max = 1.4;
theta3min = -1.8;
theta3max = 1.7;
theta4min = -1.9;
theta4max = 1.7;
theta5min = -2;
theta5max = 1.5;

i = 1;
for theta1 = theta1min:.3:theta1max
    for theta2 = theta2min:.3:theta2max
        for theta3 = theta5min:.3:theta3max
            for theta4 = theta5min:.3:theta4max
                for theta5 = theta5min:.3:theta5max
                    [jointPos,T] = calculateFK([theta1,theta2,theta3,theta4,theta5,0]);
                    x(i) = jointPos(6,1);
                    y(i) = jointPos(6,2);
                    z(i) = jointPos(6,3);
                    i = i + 1;
                end
            end
        end
    end
end

scatter3(x,y,z,'filled')

boldAxisX = [1:1:300,zeros(1,600)];
boldAxisY = [zeros(1,300),1:1:300,zeros(1,300)];
boldAxisZ = [zeros(1,600),1:1:300];
scatter3(boldAxisX,boldAxisY,boldAxisZ)
box on

xlabel('X') 
ylabel('Y')
zlabel('Z')
