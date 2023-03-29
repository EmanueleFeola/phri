%% plot simulink output
x_axis = out.xm.Time;
xm = out.xm.data;
xs = out.xs.data;
xd_s = out.xd_s.data;
dot_xm = out.dot_xm.data;
dot_xs = out.dot_xs.data;
fe = out.fe.data;
fh = out.fh.data;

figure();
subplot(3,1,1);
hold on;
grid on;
yline(xe);
plot(x_axis, xm(:,1), 'r');
plot(x_axis, xs(:,1), 'g');
plot(x_axis, xd_s(:,1), 'b');
xlabel('time [s]');
ylabel('position');
legend('xe', 'xm', 'xs', 'xd');

subplot(3,1,2);
hold on;
grid on;
plot(x_axis, dot_xm(:,1), 'g');
plot(x_axis, dot_xs(:,1), 'r');
xlabel('time [s]');
ylabel('velocity');
legend('dot xm', 'dot xs');

subplot(3,1,3);
hold on;
grid on;
plot(x_axis, fe(:,1));
plot(x_axis, fh(:,1), 'r');
xlabel('time [s]');
ylabel('force');
legend('fs=fe', 'fm=fh');

set(gcf, 'Position', get(0, 'Screensize'));
export_fig(strcat('C:\Users\emanuele\Desktop\phri_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-pdf');
% close