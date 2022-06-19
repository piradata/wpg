Controllers = ["SMC", "PID"];
graph_folder = "graps";
xPoints = {};
yPoints = {};
zPoints = {};
ptime = {};

xPointsRef = {};
yPointsRef = {};
zPointsRef = {};
ptimeRef = {};

xvelPoints = {};
yvelPoints = {};
zvelPoints = {};
veltime = {};

xvelrefPoints = {};
yvelrefPoints = {};
zvelrefPoints = {};
velreftime = {};

xPointsRef_interpolated = {};
yPointsRef_interpolated = {};
zPointsRef_interpolated = {};

BAG = {};

for i = 1:length(Controllers)

    Controller = Controllers(i);
    
    BAG{i} = rosbag(sprintf("%s_test.bag", Controller));
    
    titulo = sprintf("teste com controlador %s", Controller);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % msgStructs = readMessages(BAG,'DataFormat','struct');
    
    topic_pos = select(BAG{i}, 'Topic', '/mavros/local_position/pose');
    topic_pos_ref = select(BAG{i}, 'Topic', '/reference_pos');
    topic_vel = select(BAG{i}, 'Topic', '/mavros/local_position/velocity_local');
    topic_ref_vel = select(BAG{i}, 'Topic', '/mavros/setpoint_velocity/cmd_vel');
    topic_fuz = select(BAG{i}, 'Topic', '/wpg/fuzzy_values');
    
    data_pos = readMessages(topic_pos,'DataFormat','struct');
    data_pos_ref = readMessages(topic_pos_ref,'DataFormat','struct');
    data_vel = readMessages(topic_vel,'DataFormat','struct');
    data_ref_vel = readMessages(topic_ref_vel,'DataFormat','struct');
    data_fuz = readMessages(topic_fuz,'DataFormat','struct');
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xPoints{i} = cellfun(@(m) double(m.Pose.Position.X),data_pos);
    yPoints{i} = cellfun(@(m) double(m.Pose.Position.Y),data_pos);
    zPoints{i} = cellfun(@(m) double(m.Pose.Position.Z),data_pos);
    timeNsecPoints = cellfun(@(m) double(m.Header.Stamp.Nsec),data_pos);
    timeSecPoints = cellfun(@(m) double(m.Header.Stamp.Sec),data_pos);
    ptime{i} = (timeNsecPoints./1000000000 + timeSecPoints);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xPointsRef{i} = cellfun(@(m) double(m.Pose.Position.X),data_pos_ref);
    yPointsRef{i} = cellfun(@(m) double(m.Pose.Position.Y),data_pos_ref);
    zPointsRef{i} = cellfun(@(m) double(m.Pose.Position.Z),data_pos_ref);
    timeNsecPointsRef = cellfun(@(m) double(m.Header.Stamp.Nsec),data_pos_ref);
    timeSecPointsRef = cellfun(@(m) double(m.Header.Stamp.Sec),data_pos_ref);
    ptimeRef{i} = (timeNsecPointsRef./1000000000 + timeSecPointsRef);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xvelPoints{i} = cellfun(@(m) double(m.Twist.Linear.X),data_vel);
    yvelPoints{i} = cellfun(@(m) double(m.Twist.Linear.Y),data_vel);
    zvelPoints{i} = cellfun(@(m) double(m.Twist.Linear.Z),data_vel);
    veltimeNsecPoints = cellfun(@(m) double(m.Header.Stamp.Nsec),data_vel);
    veltimeSecPoints = cellfun(@(m) double(m.Header.Stamp.Sec),data_vel);
    veltime{i} = (veltimeNsecPoints./1000000000 + veltimeSecPoints);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xvelrefPoints{i} = cellfun(@(m) double(m.Twist.Linear.X),data_ref_vel);
    yvelrefPoints{i} = cellfun(@(m) double(m.Twist.Linear.Y),data_ref_vel);
    zvelrefPoints{i} = cellfun(@(m) double(m.Twist.Linear.Z),data_ref_vel);
    velreftimeNsecPoints = cellfun(@(m) double(m.Header.Stamp.Nsec),data_ref_vel);
    velreftimeSecPoints = cellfun(@(m) double(m.Header.Stamp.Sec),data_ref_vel);
    velreftime{i} = (velreftimeNsecPoints./1000000000 + velreftimeSecPoints);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    time = ptimeRef{i} - BAG{i}.StartTime;
    stime = time(1);
    ftime = time(end);
    sourceSize = size(xPointsRef{i});
    t = linspace(stime, ftime, sourceSize(1));
    
    time = ptime{i} - BAG{i}.StartTime;
    stime = time(1);
    ftime = time(end);
    targetSize = size(xPoints{i});
    ti = linspace(stime, ftime, targetSize(1));
    
    xPointsRef_interpolated{i} = interp1(t, xPointsRef{i}, ti)';

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    time = ptimeRef{i} - BAG{i}.StartTime;
    stime = time(1);
    ftime = time(end);
    sourceSize = size(yPointsRef{i});
    t = linspace(stime, ftime, sourceSize(1));
    
    time = ptime{i} - BAG{i}.StartTime;
    stime = time(1);
    ftime = time(end);
    targetSize = size(yPoints{i});
    ti = linspace(stime, ftime, targetSize(1));
    
    yPointsRef_interpolated{i} = interp1(t, yPointsRef{i}, ti)';

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    time = ptimeRef{i} - BAG{i}.StartTime;
    stime = time(1);
    ftime = time(end);
    sourceSize = size(zPointsRef{i});
    t = linspace(stime, ftime, sourceSize(1));
    
    time = ptime{i} - BAG{i}.StartTime;
    stime = time(1);
    ftime = time(end);
    targetSize = size(zPoints{i});
    ti = linspace(stime, ftime, targetSize(1));
    
    zPointsRef_interpolated{i} = interp1(t, zPointsRef{i}, ti)';
end

FigName = strcat("path_complete");
flower_complete_fig = figure('Name', FigName, 'Position', get(0, 'Screensize'));
plot3(xPointsRef{i},yPointsRef{i},zPointsRef{i},'DisplayName', 'Desired movement');
hold on
plot3(xPoints{1},yPoints{1},zPoints{1},'DisplayName', 'Real movement SMC');
plot3(xPoints{2},yPoints{2},zPoints{2},'DisplayName', 'Real movement PID');
title('Position tracker');
hold off;

xlabel('Absolute Position x[m]');
ylabel('Absolute Position Y[m]')
zlabel('Absolute Position Z[m]')
legend('Location', 'northeast');
set(flower_complete_fig, 'PaperPositionMode', 'auto');
exportgraphics(flower_complete_fig, strcat(fullfile(graph_folder, FigName), ".png"), 'Resolution', 900);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FigName = "all_fig";
all_fig = figure('Name', FigName, 'Position', get(0, 'Screensize'));

pos_x = subplot(3, 1, 1);
time_x_1 = ptime{1} - BAG{1}.StartTime;
points_x_1= xPoints{1} - xPointsRef_interpolated{1};
idx1 = find(~isnan(points_x_1), 1);
first_valid_time_x_1 = time_x_1(idx1);
time_x_2 = ptime{2} - BAG{2}.StartTime;
points_x_2 = xPoints{2} - xPointsRef_interpolated{2};
idx2 = find(~isnan(points_x_2), 1);
first_valid_time_x_2 = time_x_2(idx2);

plot(time_x_1 - first_valid_time_x_1, xPointsRef_interpolated{1} - xPoints{1},'DisplayName','SMC position error');
hold on
plot(time_x_2 - first_valid_time_x_2, xPointsRef_interpolated{2} - xPoints{2},'DisplayName','PID position error');
hold off
xlim([0 inf]);
xlabel('Time[s]');
ylabel('Distance Error[m]')
title('Position error X');
legend

pos_y = subplot(3, 1, 2);
time_y_1 = ptime{1} - BAG{1}.StartTime;
points_y_1= yPoints{1} - yPointsRef_interpolated{1};
idx1 = find(~isnan(points_y_1), 1);
first_valid_time_y_1 = time_y_1(idx1);
time_y_2 = ptime{2} - BAG{2}.StartTime;
points_y_2 = yPoints{2} - yPointsRef_interpolated{2};
idx2 = find(~isnan(points_y_2), 1);
first_valid_time_y_2 = time_y_2(idx2);

plot(time_y_1 - first_valid_time_y_1, yPointsRef_interpolated{1} - yPoints{1},'DisplayName','SMC position error');
hold on
plot(time_y_2 - first_valid_time_y_2, yPointsRef_interpolated{2} - yPoints{2},'DisplayName','PID position error');
hold off
xlim([0 inf]);
xlabel('Time[s]');
ylabel('Distance Error[m]')
title('Position error Y');
legend

pos_z = subplot(3, 1, 3);
time_z_1 = ptime{1} - BAG{1}.StartTime;
points_z_1= zPoints{1} - zPointsRef_interpolated{1};
idx1 = find(~isnan(points_z_1), 1);
first_valid_time_z_1 = time_z_1(idx1);
time_z_2 = ptime{2} - BAG{2}.StartTime;
points_z_2 = zPoints{2} - zPointsRef_interpolated{2};
idx2 = find(~isnan(points_z_2), 1);
first_valid_time_z_2 = time_z_2(idx2);

plot(time_z_1 - first_valid_time_z_1, zPointsRef_interpolated{1} - zPoints{1},'DisplayName','SMC position error');
hold on
plot(time_z_2 - first_valid_time_z_2, zPointsRef_interpolated{2} - zPoints{2},'DisplayName','PID position error');
hold off
xlim([0 inf]);
xlabel('Time[s]');
ylabel('Distance Error[m]')
title('Position error Z');
legend

set(all_fig, 'PaperPositionMode', 'auto');
exportgraphics(all_fig, strcat(fullfile(graph_folder, FigName), ".png"), 'Resolution', 900);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FigName = "all_fig_derivative";
all_fig_derivative = figure('Name', FigName, 'Position', get(0, 'Screensize'));

py1 = xPointsRef_interpolated{1} - xPoints{1};
px1 = time_x_1 - first_valid_time_x_1;
dy1=diff(py1)./diff(px1);
py2 = xPointsRef_interpolated{2} - xPoints{2};
px2 = time_x_2 - first_valid_time_x_2;
dy2=diff(py2)./diff(px2);

pos_x = subplot(3, 1, 1);
plot(px1(2:end),dy1,'DisplayName','SMC position error derivative');
hold on
plot(px2(2:end),dy2,'DisplayName','PID position error derivative');
hold off
xlim([0 inf]);
xlabel('Time[s]');
ylabel('Distance Error Derivative[m/s]')
title('Position error derivative X');
legend



py1 = yPointsRef_interpolated{1} - yPoints{1};
px1 = time_y_1 - first_valid_time_y_1;
dy1=diff(py1)./diff(px1);
py2 = yPointsRef_interpolated{2} - yPoints{2};
px2 = time_y_2 - first_valid_time_y_2;
dy2=diff(py2)./diff(px2);

pos_y = subplot(3, 1, 2);
plot(px1(2:end),dy1,'DisplayName','SMC position error derivative');
hold on
plot(px2(2:end),dy2,'DisplayName','PID position error derivative');
hold off
xlim([0 inf]);
xlabel('Time[s]');
ylabel('Distance Error Derivative[m/s]')
title('Position error derivative Y');
legend



py1 = zPointsRef_interpolated{1} - zPoints{1};
px1 = time_z_1 - first_valid_time_z_1;
dy1=diff(py1)./diff(px1);
py2 = zPointsRef_interpolated{2} - zPoints{2};
px2 = time_z_2 - first_valid_time_z_2;
dy2=diff(py2)./diff(px2);

pos_z = subplot(3, 1, 3);
plot(px1(2:end),dy1,'DisplayName','SMC position error derivative');
hold on
plot(px2(2:end),dy2,'DisplayName','PID position error derivative');
hold off
xlim([0 inf]);
xlabel('Time[s]');
ylabel('Distance Error Derivative[m/s]')
title('Position error derivative Z');
legend



set(all_fig_derivative, 'PaperPositionMode', 'auto');
exportgraphics(all_fig_derivative, strcat(fullfile(graph_folder, FigName), ".png"), 'Resolution', 900);
