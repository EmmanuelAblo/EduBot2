% Contrôleur MATLAB pour le SLAM en temps réel dans Webots
wb_robot_init();

% Temps de simulation
TIME_STEP = 32;

% Récupérer les dispositifs des moteurs des roues
leftMotor = wb_robot_get_device('left_wheel_joint');
rightMotor = wb_robot_get_device('right_wheel_joint');

% Configurer les moteurs pour un contrôle en vitesse continue
wb_motor_set_position(leftMotor, Inf);
wb_motor_set_position(rightMotor, Inf);

% Récupérer les capteurs d'odométrie
leftEncoder = wb_robot_get_device('left_wheel_joint_sensor');
rightEncoder = wb_robot_get_device('right_wheel_joint_sensor');

% Activer les capteurs d'odométrie
wb_position_sensor_enable(leftEncoder, TIME_STEP);
wb_position_sensor_enable(rightEncoder, TIME_STEP);

% Rayon des roues et distance entre les roues
wheelRadius = 0.0762; % Rayon des roues en mètres
wheelBase = 0.378+0.034; % Distance entre les roues en mètres

% Ajouter un capteur LiDAR
lidar = wb_robot_get_device('lidar');
wb_lidar_enable(lidar, TIME_STEP);

% Récupérer les informations du LiDAR
lidarResolution = wb_lidar_get_horizontal_resolution(lidar); 
lidarFieldOfView = wb_lidar_get_fov(lidar);
lidarAngles = linspace(-lidarFieldOfView/2, lidarFieldOfView/2, lidarResolution); 

% Attendre l'initialisation des capteurs
while isnan(wb_position_sensor_get_value(leftEncoder)) || isnan(wb_position_sensor_get_value(rightEncoder))
    wb_robot_step(TIME_STEP);
end

robot_node = wb_supervisor_node_get_self();

% Récupérer la position initiale
initial_position = wb_supervisor_node_get_position(robot_node);
initial_orientation = wb_supervisor_node_get_orientation(robot_node);

% Extraire les coordonnées X, Y, et l'angle theta
webots_initial_x = initial_position(2);
webots_initial_y = initial_position(1);
webots_initial_theta = atan2(initial_orientation(2,1), initial_orientation(1,1)) + pi/2;

% Initialisation de la position du robot
robotPose = [webots_initial_x; -webots_initial_y; -webots_initial_theta];
prevrobotPose = robotPose;
prevLeftPos = wb_position_sensor_get_value(leftEncoder);
prevRightPos = wb_position_sensor_get_value(rightEncoder);

% Activer le clavier
wb_keyboard_enable(TIME_STEP);
maxSpeed = 2; % rad/s

% Initialisation du SLAM avec des paramètres optimisés
slamAlg = lidarSLAM(20, 8); % resolution =1/20=5cm 8m
slamAlg.LoopClosureThreshold = 150; % ↓ Moins strict
slamAlg.LoopClosureSearchRadius = 6; % ↓ Réduire la zone de recherche

figure(1);
updateRate = 10; % Mettre à jour l'affichage toutes les X itérations
iteration = 0;

while wb_robot_step(TIME_STEP) ~= -1
    iteration = iteration + 1;
    tic;
    % Lire la touche pressée
    Key = wb_keyboard_get_key();
    leftSpeed = 0;
    rightSpeed = 0;
    
    switch Key
        case WB_KEYBOARD_UP
            leftSpeed = maxSpeed;
            rightSpeed = maxSpeed;
        case WB_KEYBOARD_DOWN
            leftSpeed = -maxSpeed;
            rightSpeed = -maxSpeed;
        case WB_KEYBOARD_LEFT
            leftSpeed = -maxSpeed;
            rightSpeed = maxSpeed;
        case WB_KEYBOARD_RIGHT
            leftSpeed = maxSpeed;
            rightSpeed = -maxSpeed;
        % case (key==q)
        %     break;
    end
    wb_motor_set_velocity(leftMotor, leftSpeed);
    wb_motor_set_velocity(rightMotor, rightSpeed);
    
    % Odométrie
    leftPos = wb_position_sensor_get_value(leftEncoder);
    rightPos = wb_position_sensor_get_value(rightEncoder);
    deltaLeft = (leftPos - prevLeftPos) * wheelRadius;
    deltaRight = (rightPos - prevRightPos) * wheelRadius;
    prevLeftPos = leftPos;
    prevRightPos = rightPos;
    deltaTheta = (deltaRight - deltaLeft) / wheelBase;
    deltaX = ((deltaRight + deltaLeft) / 2) * cos(robotPose(3));
    deltaY = ((deltaRight + deltaLeft) / 2) * sin(robotPose(3));
    robotPose = robotPose + [deltaX; deltaY; deltaTheta];
    relrobotPose = prevrobotPose - robotPose;
    % Acquisition LiDAR et filtrage des scans
    lidarRanges = wb_lidar_get_range_image(lidar);
    
    if ~isempty(lidarRanges)
        cart = zeros(length(lidarRanges), 2);
        for i = 1:length(lidarRanges)
            r = lidarRanges(i);
            if r > 0 && r < 10 % ↓ Filtrer les valeurs aberrantes
                cart(i, :) = [r * cos(lidarAngles(i))-0.25, -r * sin(lidarAngles(i))];%
            end
        end
        
        if any(cart(:, 1) ~= 0)
            scan = lidarScan(cart);
            
            % Vérifier si le robot a suffisamment bougé avant d'ajouter un scan
            if norm([deltaX, deltaY]) > 0.02 || abs(deltaTheta) > 0.02 
                addScan(slamAlg, scan);
            end
        end
    end
    
    clf(1);
    show(slamAlg);
    hold on;
    show(slamAlg.PoseGraph);
    hold off;

    pause(0.01);
    prevrobotPose = robotPose;

toc
end

[scansSLAM,poses] = scansAndPoses(slamObj);
myOccMap = buildMap(scansSLAM,poses,resolution,maxRange);
save('occupancy_map.mat', 'myOccMap');
title('Occupancy Map')