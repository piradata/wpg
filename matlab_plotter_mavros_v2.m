Controllers = ["SMC_PID_8", "SMC_PID_8_WW", "PID", "PID_WW"];
%Controllers = ["SMC_PID_8_WW", "PID_WW"];
%Controllers = ["SMC_PID_8","PID"];
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
    
    BAG{i} = rosbag(sprintf("%s.bag", Controller));
    
    titulo = sprintf("teste com controlador %s", Controller);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % msgStructs = readMessages(BAG,'DataFormat','struct');
    
    topic_pos = select(BAG{i}, 'Topic', '/mavros/local_position/pose');
    topic_pos_ref = select(BAG{i}, 'Topic', '/reference_pos');
    topic_vel = select(BAG{i}, 'Topic', '/mavros/local_position/velocity_local');
    topic_ref_vel = select(BAG{i}, 'Topic', '/mavros/setpoint_velocity/cmd_vel');
    topic_fuz = select(BAG{i}, 'Topic', '/wpg/fuzzy_values');
    topic_sensor_angacc = select(BAG{i}, 'Topic', '/mavros/local_position/odom');
    
    data_pos = readMessages(topic_pos,'DataFormat','struct');
    data_pos_ref = readMessages(topic_pos_ref,'DataFormat','struct');
    data_vel = readMessages(topic_vel,'DataFormat','struct');
    data_ref_vel = readMessages(topic_ref_vel,'DataFormat','struct');
    data_sensor_angacc = readMessages(topic_sensor_angacc,'DataFormat','struct');
    
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
    xAngAccelPoints{i} = cellfun(@(m) double(m.Twist.Twist.Angular.X),data_sensor_angacc);
    yAngAccelPoints{i} = cellfun(@(m) double(m.Twist.Twist.Angular.Y),data_sensor_angacc);
    zAngAccelPoints{i} = cellfun(@(m) double(m.Twist.Twist.Angular.Z),data_sensor_angacc);
    AngAcceltimeNsecPoints = cellfun(@(m) double(m.Header.Stamp.Nsec),data_sensor_angacc);
    AngAcceltimeSecPoints = cellfun(@(m) double(m.Header.Stamp.Sec),data_sensor_angacc);
    AngAcceltime{i} = (AngAcceltimeNsecPoints./1000000000 + AngAcceltimeSecPoints);

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




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Path plot

FigName = strcat("path");
path_complete_fig = figure('Name', FigName, 'Position', get(0, 'Screensize'));
plot3(xPointsRef{1},yPointsRef{1},zPointsRef{1},'DisplayName', 'Desired movement');
hold on
for controller = 1:length(Controllers)
    Controller = Controllers(controller);
    plot3(xPoints{controller},yPoints{controller},zPoints{controller},'DisplayName', Controller);
end
title('Position tracker');
hold off;

xlabel('Absolute Position x[m]');
ylabel('Absolute Position Y[m]')
zlabel('Absolute Position Z[m]')
legend('Location', 'northeast', 'interpreter', 'none');
set(path_complete_fig, 'PaperPositionMode', 'auto');
exportgraphics(path_complete_fig, strcat(fullfile(graph_folder, FigName), ".png"), 'Resolution', 900);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FigName = "rate_error";
all_fig = figure('Name', FigName, 'Position', get(0, 'Screensize'));
for axis = 1:3
    for path = 1:3
        subplot(3, 3, path + axis*3 - 3);
        hold off
        for controller = 1:length(Controllers)
            Controller = Controllers(controller);
            if axis == 1
                my_title='Position error on X' + " ";
                my_PointsRef_interpolated = xPointsRef_interpolated{controller};
                my_Points = xPoints{controller};
            elseif axis == 2
                my_title='Position error on Y' + " ";
                my_PointsRef_interpolated = yPointsRef_interpolated{controller};
                my_Points = yPoints{controller};
            elseif axis == 3
                my_title='Position error on Z' + " ";
                my_PointsRef_interpolated = zPointsRef_interpolated{controller};
                my_Points = zPoints{controller};
            end
            change_1 = find(abs(diff(xPointsRef_interpolated{controller} - xPoints{controller}))>0.5);
            change_2 = find(abs(diff(zPointsRef_interpolated{controller} - xPoints{controller}))>0.5);
            change_1_start = change_1(1)   -2;
            change_1_end   = change_1(end) +2;
            change_2_start = change_2(1)   -2;
            change_2_end   = change_2(end) +2;
            
            time = ptime{controller} - BAG{controller}.StartTime;
            points = my_Points - my_PointsRef_interpolated;
            idx1 = find(~isnan(points), 1);
            first_valid_time = time(idx1);
            if path == 1
                path_mode = 'on takeoff';
                plot(time(1:change_1_start) - first_valid_time, my_PointsRef_interpolated(1:change_1_start) - my_Points(1:change_1_start),'DisplayName',Controller);
            elseif path == 2
                path_mode = 'on move';
                plot(time(change_1_end:change_2_start) - first_valid_time, my_PointsRef_interpolated(change_1_end:change_2_start) - my_Points(change_1_end:change_2_start),'DisplayName',Controller);
            elseif path == 3
                path_mode = 'on landing';
                plot(time(change_2_end:end) - first_valid_time, my_PointsRef_interpolated(change_2_end:end) - my_Points(change_2_end:end),'DisplayName',Controller);
            end
            title(my_title + path_mode);
            xlabel('Time[s]');
            ylabel('Distance Error[m]')
            legend('interpreter', 'none')
            hold on
        end
    end
end
set(all_fig, 'PaperPositionMode', 'auto');
exportgraphics(all_fig, strcat(fullfile(graph_folder, FigName), ".png"), 'Resolution', 900);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FigName = "rate_error_derivative";
all_fig_derivative = figure('Name', FigName, 'Position', get(0, 'Screensize'));
for axis = 1:3
    for path = 1:3
        subplot(3, 3, path + axis*3 - 3);
        hold off
        for controller = 1:length(Controllers)
            Controller = Controllers(controller);
            if axis == 1
                my_title='Absolute derivative of position error on X' + " ";
                my_PointsRef_interpolated = xPointsRef_interpolated{controller};
                my_Points = xPoints{controller};
            elseif axis == 2
                my_title='Absolute derivative of position error on Y' + " ";
                my_PointsRef_interpolated = yPointsRef_interpolated{controller};
                my_Points = yPoints{controller};
            elseif axis == 3
                my_title='Absolute derivative of position error on Z' + " ";
                my_PointsRef_interpolated = zPointsRef_interpolated{controller};
                my_Points = zPoints{controller};
            end
            change_1 = find(abs(diff(xPointsRef_interpolated{controller} - xPoints{controller}))>0.5);
            change_2 = find(abs(diff(zPointsRef_interpolated{controller} - xPoints{controller}))>0.5);
            change_1_start = change_1(1)   -2;
            change_1_end   = change_1(end) +2;
            change_2_start = change_2(1)   -2;
            change_2_end   = change_2(end) +2;
            
            time = ptime{controller} - BAG{controller}.StartTime;
            points= my_Points - my_PointsRef_interpolated;
            idx1 = find(~isnan(points), 1);
            first_valid_time = time(idx1);

            py1 = my_PointsRef_interpolated - my_Points;
            px1 = time - first_valid_time;
            dy1=diff(py1)./diff(px1);

            display_name = Controller;
            if path == 1
                path_mode = '- takeoff';
                plot(px1(1:change_1_start),abs(dy1(1:change_1_start)),'DisplayName',display_name);
            elseif path == 2
                path_mode = '- move';
                plot(px1(change_1_end:change_2_start),abs(dy1(change_1_end:change_2_start)),'DisplayName',display_name);
            elseif path == 3
                path_mode = '- landing';
                plot(px1(change_2_end:end-1),abs(dy1(change_2_end:end)),'DisplayName',display_name);
            end
            title(my_title + path_mode);
            xlabel('Time[s]');
            ylabel('Distance Error Derivative[m]')
            legend('interpreter', 'none')
            hold on
        end
    end
end
set(all_fig_derivative, 'PaperPositionMode', 'auto');
exportgraphics(all_fig_derivative, strcat(fullfile(graph_folder, FigName), ".png"), 'Resolution', 900);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Sensor plot

FigName = strcat("rate_variation_sensoring");
sensors_fig = figure('Name', FigName, 'Position', get(0, 'Screensize'));

subplot(3,1,1)
plot(AngAcceltime{1} - BAG{1}.StartTime,xAngAccelPoints{1},'DisplayName', 'Ang Accel X SMC with wind');
hold on
plot(AngAcceltime{2} - BAG{2}.StartTime,xAngAccelPoints{2},'DisplayName', 'Ang Accel X PID with wind');
hold off
title('angular acceleration about the FRD body frame XYZ-axis in rad/s^2');
xlabel('Time t[s]');
ylabel('Sensor data ang accel [rad/s^2]')
legend('Location', 'northeast');

subplot(3,1,2)
plot(AngAcceltime{1} - BAG{1}.StartTime,yAngAccelPoints{1},'DisplayName', 'Ang Accel Y SMC with wind');
hold on
plot(AngAcceltime{2} - BAG{2}.StartTime,yAngAccelPoints{2},'DisplayName', 'Ang Accel Y PID with wind');
hold off
title('angular acceleration about the FRD body frame XYZ-axis in rad/s^2');
xlabel('Time t[s]');
ylabel('Sensor data ang accel [rad/s^2]')
legend('Location', 'northeast');

subplot(3,1,3)
plot(AngAcceltime{1} - BAG{1}.StartTime,zAngAccelPoints{1},'DisplayName', 'Ang Accel Z SMC with wind');
hold on
plot(AngAcceltime{2} - BAG{2}.StartTime,zAngAccelPoints{2},'DisplayName', 'Ang Accel Z PID with wind');
hold off
title('angular acceleration about the FRD body frame XYZ-axis');
xlabel('Time t[s]');
ylabel('Sensor data ang accel [rad/s^2]')
legend('Location', 'northeast', 'interpreter', 'none');


set(sensors_fig, 'PaperPositionMode', 'auto');
exportgraphics(sensors_fig, strcat(fullfile(graph_folder, FigName), ".png"), 'Resolution', 900);

