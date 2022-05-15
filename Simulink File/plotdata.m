

% Responses figure setup
f_x = figure;
f_y = figure;
f_z = figure;

f_roll = figure;
f_pitch = figure;
f_yaw = figure;

f_traj = figure();

f_motor = figure();


% Plot x response
figure(f_x);
plot(out.x);
hold on;
plot(out.des_x);
legend('x', 'desired x')
xlabel('time (s)')
ylabel('x (m)')
title('x response')
ax =gca;
ax.FontSize = 13;


%Plot y response
figure(f_y);
plot(out.y);
hold on;
plot(out.des_y);
legend('y', 'desired y')
xlabel('time (s)')
ylabel('y (m)')
title('y response')
ax =gca;
ax.FontSize = 13;

%Plot z response
figure(f_z);
plot(out.z);
hold on;
plot(out.des_z);
legend('z', 'desired z')
xlabel('time (s)')
ylabel('z (m)')
title('z response')
ax =gca;
ax.FontSize = 13;

%plot roll response
figure(f_roll);
plot(out.roll);
hold on;
plot(out.des_roll);
legend('roll', 'desired roll')
xlabel('time (s)')
ylabel('roll (rad)')
title('roll response')
ax =gca;
ax.FontSize = 13;

%Plot pitch response
figure(f_pitch);
plot(out.pitch);
hold on;
plot(out.des_pitch);
legend('pitch', 'desired pitch')
xlabel('time (s)')
ylabel('pitch (rad)')
title('pitch response')
ax =gca;
ax.FontSize = 13;

%Plot yaw response
figure(f_yaw);
plot(out.yaw);
hold on;
plot(out.des_yaw);
legend('yaw', 'desired yaw')
xlabel('time (s)')
ylabel('yaw (rad)')
title('yaw response')
ax =gca;
ax.FontSize = 13;

%Plot 3d trajectory
figure(f_traj);
plot3(out.x.data, out.y.data, out.z.data)
legend('x', 'y', 'z')
xlabel('x (m)')
ylabel('y (m)')
title('z (m)')
ax =gca;
ax.FontSize = 13;

%Plot Motor Velocity

figure(f_motor);
subplot(2,2,1);
plot(out.motor1);
title('Motor 1');
xlabel('time (s)')
ylabel('angular vel (rad/s)')
ax =gca;
ax.FontSize = 13;

subplot(2,2,2);
plot(out.motor2);
title('Motor 2');
xlabel('time (s)')
ylabel('angular vel (rad/s)')
ax =gca;
ax.FontSize = 13;

subplot(2,2,3);
plot(out.motor3);
title('Motor 3');
xlabel('time (s)')
ylabel('angular vel (rad/s)')
ax =gca;
ax.FontSize = 13;

subplot(2,2,4);
plot(out.motor4);
title('Motor 4');
xlabel('time (s)')
ylabel('angular vel (rad/s)')
ax =gca;
ax.FontSize = 13;


%save results

saveas(f_x, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\x_response');
saveas(f_x, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\x_response.bmp');

saveas(f_y, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\y_response');
saveas(f_y, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\y_response.bmp');

saveas(f_z, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\z_response');
saveas(f_z, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\z_response.bmp');

saveas(f_roll, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\roll_response');
saveas(f_roll, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\roll_response.bmp');

saveas(f_pitch, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\pitch_response');
saveas(f_pitch, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\pitch_response.bmp');

saveas(f_yaw, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\yaw_response');
saveas(f_yaw, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\yaw_response.bmp');

saveas(f_traj, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\traj_response');
saveas(f_traj, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\traj_response.bmp');

saveas(f_motor, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\motor_vel');
saveas(f_motor, 'C:\Users\user\Desktop\UGRP\모델링, 제어\Testdata\motor_vel.bmp');
