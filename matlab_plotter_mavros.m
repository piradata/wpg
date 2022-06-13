Controllers = ["SMC", "PID"];
graph_folder = "graps";
for i = 1:length(Controllers)

    Controller = Controllers(i);
    
    BAG = rosbag(sprintf("%s_test.bag", Controller));
    
    titulo = sprintf("teste com controlador %s", Controller);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % msgStructs = readMessages(BAG,'DataFormat','struct');
    
    topic_pos = select(BAG, 'Topic', '/mavros/local_position/pose');
    topic_pos_ref = select(BAG, 'Topic', '/reference_pos');
    topic_vel = select(BAG, 'Topic', '/mavros/local_position/velocity_local');
    topic_ref_vel = select(BAG, 'Topic', '/mavros/setpoint_velocity/cmd_vel');
    topic_fuz = select(BAG, 'Topic', '/wpg/fuzzy_values');
    
    data_pos = readMessages(topic_pos,'DataFormat','struct');
    data_pos_ref = readMessages(topic_pos_ref,'DataFormat','struct');
    data_vel = readMessages(topic_vel,'DataFormat','struct');
    data_ref_vel = readMessages(topic_ref_vel,'DataFormat','struct');
    data_fuz = readMessages(topic_fuz,'DataFormat','struct');
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xPoints = cellfun(@(m) double(m.Pose.Position.X),data_pos);
    yPoints = cellfun(@(m) double(m.Pose.Position.Y),data_pos);
    zPoints = cellfun(@(m) double(m.Pose.Position.Z),data_pos);
    timeNsecPoints = cellfun(@(m) double(m.Header.Stamp.Nsec),data_pos);
    timeSecPoints = cellfun(@(m) double(m.Header.Stamp.Sec),data_pos);
    ptime = (timeNsecPoints./1000000000 + timeSecPoints);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xPointsRef = cellfun(@(m) double(m.Pose.Position.X),data_pos_ref);
    yPointsRef = cellfun(@(m) double(m.Pose.Position.Y),data_pos_ref);
    zPointsRef = cellfun(@(m) double(m.Pose.Position.Z),data_pos_ref);
    timeNsecPointsRef = cellfun(@(m) double(m.Header.Stamp.Nsec),data_pos_ref);
    timeSecPointsRef = cellfun(@(m) double(m.Header.Stamp.Sec),data_pos_ref);
    ptimeRef = (timeNsecPointsRef./1000000000 + timeSecPointsRef);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xvelPoints = cellfun(@(m) double(m.Twist.Linear.X),data_vel);
    yvelPoints = cellfun(@(m) double(m.Twist.Linear.Y),data_vel);
    zvelPoints = cellfun(@(m) double(m.Twist.Linear.Z),data_vel);
    veltimeNsecPoints = cellfun(@(m) double(m.Header.Stamp.Nsec),data_vel);
    veltimeSecPoints = cellfun(@(m) double(m.Header.Stamp.Sec),data_vel);
    veltime = (veltimeNsecPoints./1000000000 + veltimeSecPoints);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xvelrefPoints = cellfun(@(m) double(m.Twist.Linear.X),data_ref_vel);
    yvelrefPoints = cellfun(@(m) double(m.Twist.Linear.Y),data_ref_vel);
    zvelrefPoints = cellfun(@(m) double(m.Twist.Linear.Z),data_ref_vel);
    velreftimeNsecPoints = cellfun(@(m) double(m.Header.Stamp.Nsec),data_ref_vel);
    velreftimeSecPoints = cellfun(@(m) double(m.Header.Stamp.Sec),data_ref_vel);
    velreftime = (velreftimeNsecPoints./1000000000 + velreftimeSecPoints);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    FigName = strcat(Controller, "_", "all");
    all_fig = figure('Name', FigName,'Position', get(0, 'Screensize'));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pos_x = subplot(3, 3, 1);
    xlabel('x-axis');
    plot(ptime - BAG.StartTime, xPoints,'DisplayName','X position');
    hold on;
    plot(ptimeRef - BAG.StartTime, xPointsRef,'DisplayName','X ref position');
    xlim([0 inf]);
    title('Position X');
    hold off;
    legend;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pos_y = subplot(3, 3, 2);
    xlabel('y-axis');
    plot(ptime - BAG.StartTime, yPoints,'DisplayName','Y position');
    hold on
    plot(ptimeRef - BAG.StartTime, yPointsRef,'DisplayName','Y ref position');
    xlim([0 inf]);
    title('Position Y');
    hold off
    legend
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pos_z = subplot(3, 3, 3);
    xlabel('z-axis');
    plot(ptime - BAG.StartTime, zPoints,'DisplayName','Z position');
    hold on;
    plot(ptimeRef - BAG.StartTime, zPointsRef,'DisplayName','Z ref position');
    xlim([0 inf]);
    title('Position Z');
    hold off;
    legend;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    vel_x = subplot(3, 3, 4);
    xlabel('x-axis');
    plot(veltime - BAG.StartTime, xvelPoints,'DisplayName','X Velocity');
    hold on;
    plot(velreftime - BAG.StartTime, xvelrefPoints,'DisplayName','X Velocity REF');
    xlim([0 inf]);
    title('Velocity X');
    hold off;
    legend;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    vel_y = subplot(3, 3, 5);
    xlabel('y-axis');
    plot(veltime - BAG.StartTime, yvelPoints,'DisplayName','Y Velocity');
    hold on;
    plot(velreftime - BAG.StartTime, yvelrefPoints,'DisplayName','Y Velocity REF');
    xlim([0 inf]);
    title('Velocity Y');
    hold off;
    legend;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    vel_z = subplot(3, 3, 6);
    xlabel('z-axis');
    plot(veltime - BAG.StartTime, zvelPoints,'DisplayName','Z Velocity');
    hold on;
    plot(velreftime - BAG.StartTime, zvelrefPoints,'DisplayName','Z Velocity REF');
    xlim([0 inf]);
    title('Velocity Z');
    hold off;
    legend;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    err_pos_x = subplot(3, 3, 7);
    xlabel('x-axis');
    xlim([0 inf]);
    
    time = ptimeRef - BAG.StartTime;
    stime = time(1);
    ftime = time(end);
    sourceSize = size(xPointsRef);
    t = linspace(stime,ftime,sourceSize(1));
    
    time = ptime - BAG.StartTime;
    stime = time(1);
    ftime = time(end);
    targetSize = size(xPoints);
    ti = linspace(stime,ftime,targetSize(1));
    
    xPointsRef_interpolated = interp1(t,xPointsRef,ti)';
    plot(ptime - BAG.StartTime, xPointsRef_interpolated - xPoints,'DisplayName','X position error');
    xlim([0 inf]);
    title('Position error X');
    legend
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    err_pos_y = subplot(3, 3, 8);
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
    plot(ptime - BAG.StartTime, yPointsRef_interpolated - yPoints,'DisplayName','Y position error');
    xlim([0 inf]);
    title('Position error Y');
    legend
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    err_pos_z = subplot(3, 3, 9);
    xlabel('z-axis');
    xlim([0 inf]);
    
    time = ptimeRef - BAG.StartTime;
    stime = time(1);
    ftime = time(end);
    sourceSize = size(zPointsRef);
    t = linspace(stime,ftime,sourceSize(1));
    
    time = ptime - BAG.StartTime;
    stime = time(1);
    ftime = time(end);
    targetSize = size(zPoints);
    ti = linspace(stime,ftime,targetSize(1));
    
    zPointsRef_interpolated = interp1(t,zPointsRef,ti)';
    plot(ptime - BAG.StartTime, zPointsRef_interpolated - zPoints,'DisplayName','Z position error');
    xlim([0 inf]);
    title('Position error Z');
    legend
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    set(all_fig, 'PaperPositionMode', 'auto');
    exportgraphics(all_fig, strcat(fullfile(graph_folder, FigName), ".png"), 'Resolution', 900);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    hfig = figure;
    hax_new = copyobj(pos_x, hfig);
    set(hax_new, 'Position', get(0, 'DefaultAxesPosition'));
    set(hfig, 'PaperPositionMode', 'auto');
    legend(hax_new, 'Location', 'southeast');
    xlabel('Time[s]');
    ylabel('Position[m]');
    exportgraphics(hfig, strcat(fullfile(graph_folder, strcat(Controller, "_posx")), ".png"), 'Resolution', 600);

    hfig = figure;
    hax_new = copyobj(pos_y, hfig);
    set(hax_new, 'Position', get(0, 'DefaultAxesPosition'));
    set(hfig, 'PaperPositionMode', 'auto');
    legend(hax_new, 'Location', 'southeast');
    xlabel('Time[s]');
    ylabel('Position[m]');
    exportgraphics(hfig, strcat(fullfile(graph_folder, strcat(Controller, "_posy")), ".png"), 'Resolution', 600);

    hfig = figure;
    hax_new = copyobj(pos_z, hfig);
    set(hax_new, 'Position', get(0, 'DefaultAxesPosition'));
    set(hfig, 'PaperPositionMode', 'auto');
    legend(hax_new, 'Location', 'northwest');
    xlabel('Time[s]');
    ylabel('Position[m]');
    exportgraphics(hfig, strcat(fullfile(graph_folder, strcat(Controller, "_posz")), ".png"), 'Resolution', 600);

    hfig = figure;
    hax_new = copyobj(vel_x, hfig);
    set(hax_new, 'Position', get(0, 'DefaultAxesPosition'));
    set(hfig, 'PaperPositionMode', 'auto');
    legend(hax_new, 'Location', 'northwest');
    xlabel('Time[s]');
    ylabel('Velocity[m/s]');
    exportgraphics(hfig, strcat(fullfile(graph_folder, strcat(Controller, "_velx")), ".png"), 'Resolution', 600);

    hfig = figure;
    hax_new = copyobj(vel_y, hfig);
    set(hax_new, 'Position', get(0, 'DefaultAxesPosition'));
    set(hfig, 'PaperPositionMode', 'auto');
    legend(hax_new, 'Location', 'northwest');
    xlabel('Time[s]');
    ylabel('Velocity[m/s]');
    exportgraphics(hfig, strcat(fullfile(graph_folder, strcat(Controller, "_vely")), ".png"), 'Resolution', 600);

    hfig = figure;
    hax_new = copyobj(vel_z, hfig);
    set(hax_new, 'Position', get(0, 'DefaultAxesPosition'));
    set(hfig, 'PaperPositionMode', 'auto');
    legend(hax_new, 'Location', 'northwest');
    xlabel('Time[s]');
    ylabel('Velocity[m/s]');
    exportgraphics(hfig, strcat(fullfile(graph_folder, strcat(Controller, "_velz")), ".png"), 'Resolution', 600);

    hfig = figure;
    hax_new = copyobj(err_pos_x, hfig);
    set(hax_new, 'Position', get(0, 'DefaultAxesPosition'));
    set(hfig, 'PaperPositionMode', 'auto');
    legend(hax_new, 'Location', 'southwest');
    xlabel('Time[s]');
    ylabel('Position Error[m]');
    exportgraphics(hfig, strcat(fullfile(graph_folder, strcat(Controller, "_poserrx")), ".png"), 'Resolution', 600);

    hfig = figure;
    hax_new = copyobj(err_pos_y, hfig);
    set(hax_new, 'Position', get(0, 'DefaultAxesPosition'));
    set(hfig, 'PaperPositionMode', 'auto');
    legend(hax_new, 'Location', 'southwest');
    xlabel('Time[s]');
    ylabel('Position Error[m]');
    exportgraphics(hfig, strcat(fullfile(graph_folder, strcat(Controller, "_poserry")), ".png"), 'Resolution', 600);

    hfig = figure;
    hax_new = copyobj(err_pos_z, hfig);
    set(hax_new, 'Position', get(0, 'DefaultAxesPosition'));
    set(hfig, 'PaperPositionMode', 'auto');
    legend(hax_new, 'Location', 'northwest');
    xlabel('Time[s]');
    ylabel('Position Error[m]');
    exportgraphics(hfig, strcat(fullfile(graph_folder, strcat(Controller, "_poserrz")), ".png"), 'Resolution', 600);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    FigName = strcat(Controller, "_", "flower");
    
    flower_fig = figure('Name', FigName);
    plot3(xPoints,yPoints,zPoints,'DisplayName', 'Real movement');
    hold on;
    plot3(xPointsRef,yPointsRef,zPointsRef,'DisplayName', 'Desired movement');
    xlabel('X position[m]');
    xh = get(flower_fig.CurrentAxes,'XLabel'); % Handle of the x label
    set(xh, 'Units', 'Normalized');
    pos = get(xh, 'Position');
    set(xh, 'Position',pos.*[1,1,1],'Rotation',22);
    xtickangle(-30);
    ylabel('Y position[m]');
    yh = get(flower_fig.CurrentAxes,'YLabel'); % Handle of the y label
    set(yh, 'Units', 'Normalized');
    pos = get(yh, 'Position');
    set(yh, 'Position',pos.*[1,1,1],'Rotation',-22);
    ytickangle(30);
    zlabel('Z position[m]');
    xticks(-2:0.5:2);
    yticks(-2:0.5:2);
    zticks(0:0.5:4);
    title(sprintf('Position tracker %s', Controller));
    hold off;
    legend('Location', 'northeast');

    set(flower_fig, 'PaperPositionMode', 'auto');
    exportgraphics(flower_fig, strcat(fullfile(graph_folder, FigName), ".png"), 'Resolution', 900);

end
close all
