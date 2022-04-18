err_pos_y = figure('Position', get(0, 'Screensize'));
xlabel('y-axis');
xlim([0 inf]);

time = ptimeRef - BAG.StartTime;
stime = time(1);
ftime = time(end);
sourceSize = size(yPointsRef);
t = linspace(stime,ftime,sourceSize(1));

time = ptime - BAG.StartTime;
stime = time(1);
ftime = time(end);
targetSize = size(yPoints);
ti = linspace(stime,ftime,targetSize(1));

yPointsRef_interpolated = interp1(t,yPointsRef,ti)';
plot(ptime - BAG.StartTime, yPoints - yPointsRef_interpolated,'DisplayName','Y position error');
hold on
plot(ptime - BAG.StartTime, yPoints,'DisplayName','Y position');
plot(ptime - BAG.StartTime, yPointsRef_interpolated,'DisplayName','Y position REF interpolated');
hold off
xlim([0 inf]);
title('Position error Y');
legend