%P = rosbag("P_4.bag");
%PD = rosbag("PD_3_0.08.bag");
%PI = rosbag("PI_3.2_0.95.bag");
%PID1 = rosbag("PID_2.60_0.25_0.40.bag");
%PID2 = rosbag("PID_3.255_4.912_0.565.bag");
%REAL = rosbag("oitoGB.bag");

BAG = PID2;
titulo = "teste com controlador PID";

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% msgStructs = readMessages(BAG,'DataFormat','struct');

topic_pos = select(BAG, 'Topic', '/mavros/local_position/pose');
topic_vel2 = select(BAG, 'Topic', '/mavros/local_position/velocity_local');
topic_ref_vel = select(BAG, 'Topic', '/mavros/setpoint_velocity/cmd_vel');

data_pos = readMessages(topic_pos,'DataFormat','struct');
data_vel2 = readMessages(topic_vel2,'DataFormat','struct');
data_ref_vel = readMessages(topic_ref_vel,'DataFormat','struct');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xPoints = cellfun(@(m) double(m.Pose.Position.X),data_pos);
yPoints = cellfun(@(m) double(m.Pose.Position.Y),data_pos);
zPoints = cellfun(@(m) double(m.Pose.Position.Z),data_pos);
timeNsecPoints = cellfun(@(m) double(m.Header.Stamp.Nsec),data_pos);
timeSecPoints = cellfun(@(m) double(m.Header.Stamp.Sec),data_pos);
ptime = (timeNsecPoints./1000000000 + timeSecPoints);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xvelPoints = cellfun(@(m) double(m.Twist.Linear.X),data_vel2);
yvelPoints = cellfun(@(m) double(m.Twist.Linear.Y),data_vel2);
zvelPoints = cellfun(@(m) double(m.Twist.Linear.Z),data_vel2);
veltimeNsecPoints = cellfun(@(m) double(m.Header.Stamp.Nsec),data_vel2);
veltimeSecPoints = cellfun(@(m) double(m.Header.Stamp.Sec),data_vel2);
veltime = (veltimeNsecPoints./1000000000 + veltimeSecPoints);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xvelrefPoints = cellfun(@(m) double(m.Twist.Linear.X),data_ref_vel);
yvelrefPoints = cellfun(@(m) double(m.Twist.Linear.Y),data_ref_vel);
zvelrefPoints = cellfun(@(m) double(m.Twist.Linear.Z),data_ref_vel);
velreftimeNsecPoints = cellfun(@(m) double(m.Header.Stamp.Nsec),data_ref_vel);
velreftimeSecPoints = cellfun(@(m) double(m.Header.Stamp.Sec),data_ref_vel);
velreftime = (velreftimeNsecPoints./1000000000 + velreftimeSecPoints);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(4, 1, 1)
xlabel('x-axis')
xlim([0 inf]) 
plot(ptime - BAG.StartTime, xPoints,'DisplayName','X position');
hold on
plot(ptime - BAG.StartTime, yPoints,'DisplayName','Y position');
plot(ptime - BAG.StartTime, zPoints,'DisplayName','Z position');
xlim([0 inf]) 
title('Posição')
hold off
legend

subplot(4, 1, 2)
plot(veltime - BAG.StartTime, xvelPoints,'DisplayName','X Velocity');
hold on
plot(velreftime - BAG.StartTime, xvelrefPoints,'DisplayName','X Velocity REF');
xlim([0 inf]) 
title('Velocidade em X')
hold off
legend

subplot(4, 1, 3)
plot(veltime - BAG.StartTime, yvelPoints,'DisplayName','Y Velocity');
hold on
plot(velreftime - BAG.StartTime, yvelrefPoints,'DisplayName','Y Velocity REF');
xlim([0 inf]) 
title('Velocidade em Y')
hold off
legend

subplot(4, 1, 4)
plot(veltime - BAG.StartTime, zvelPoints,'DisplayName','Z Velocity');
hold on
plot(velreftime - BAG.StartTime, zvelrefPoints,'DisplayName','Z Velocity REF');
xlim([0 inf]) 
title('Velocidade em Z')
hold off
legend

sgtitle(titulo)