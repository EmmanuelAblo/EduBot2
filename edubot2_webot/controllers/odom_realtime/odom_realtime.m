load('occupancy_map.mat','myOccMap')
% Contrôleur MATLAB pour le robot Eduro dans Webots
wb_robot_init();

% Temps de simulation
TIME_STEP = 40;

% Récupérer la caméra RGB et la caméra de profondeur
camera = wb_robot_get_device('rgb_camera');
depthCamera = wb_robot_get_device('depth_camera');

% Activer la caméra et la caméra de profondeur
wb_camera_enable(camera, TIME_STEP);
wb_range_finder_enable(depthCamera, TIME_STEP);

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
wheelBase = 0.378+0.035; % Distance entre les roues en mètres (0.2 * 2)

% Ajouter un capteur LiDAR
lidar = wb_robot_get_device('lidar');
wb_lidar_enable(lidar, TIME_STEP);

% Récupérer les informations du LiDAR
lidarResolution = wb_lidar_get_horizontal_resolution(lidar); 
lidarFieldOfView = wb_lidar_get_fov(lidar);
lidarAngles = linspace(-lidarFieldOfView/2, lidarFieldOfView/2, lidarResolution); 

% Attendre que les capteurs de position soient initialisés
while isnan(wb_position_sensor_get_value(leftEncoder)) || isnan(wb_position_sensor_get_value(rightEncoder))
    wb_robot_step(TIME_STEP);
end

robot_node = wb_supervisor_node_get_self();

% Récupérer la position initiale
initial_position = wb_supervisor_node_get_position(robot_node);
initial_orientation = wb_supervisor_node_get_orientation(robot_node); % Matrice 3x3

% Extraire les coordonnées X, Y, et l'angle theta
webots_initial_x = initial_position(1); % Position X sur webots
webots_initial_y = initial_position(2); % Position Y sur webots
webots_initial_theta = atan2(initial_orientation(2,1), initial_orientation(1,1)) + pi/2 ; % Calcul de l'angle

% Conversion vers le repère occupancyMap
robotPose = [webots_initial_y; -webots_initial_x; -webots_initial_theta ];
prevLeftPos = wb_position_sensor_get_value(leftEncoder);
prevRightPos = wb_position_sensor_get_value(rightEncoder);

% Activer le clavier
wb_keyboard_enable(TIME_STEP);
maxSpeed = 3;% rad/s

% figure(1)
% Boucle principale de la simulation
while wb_robot_step(TIME_STEP) ~= -1
    % Lire la touche pressée
    Key = wb_keyboard_get_key();
    
    % Initialiser les vitesses des moteurs
    leftSpeed = 0;
    rightSpeed = 0;
    
    % Contrôle du robot avec le clavier
    switch Key
        case WB_KEYBOARD_UP  % Flèche haut
            leftSpeed = maxSpeed;
            rightSpeed = maxSpeed;
        case WB_KEYBOARD_DOWN  % Flèche bas
            leftSpeed = -maxSpeed;
            rightSpeed = -maxSpeed;
        case WB_KEYBOARD_LEFT  % Flèche gauche
            leftSpeed = -maxSpeed;
            rightSpeed = maxSpeed;
        case WB_KEYBOARD_RIGHT  % Flèche droite
            leftSpeed = maxSpeed;
            rightSpeed = -maxSpeed;
    end
    % Appliquer les vitesses aux moteurs
    wb_motor_set_velocity(leftMotor, leftSpeed);
    wb_motor_set_velocity(rightMotor, rightSpeed);

    % Mise à jour de l'odométrie
    leftPos = wb_position_sensor_get_value(leftEncoder);
    rightPos = wb_position_sensor_get_value(rightEncoder);

    deltaLeft = (leftPos - prevLeftPos) * wheelRadius;
    deltaRight = (rightPos - prevRightPos) * wheelRadius;

    prevLeftPos = leftPos;
    prevRightPos = rightPos;

    deltaTheta = (deltaRight - deltaLeft) / wheelBase;
    deltaX = ((deltaRight + deltaLeft) / 2) * cos(robotPose(3));
    deltaY = ((deltaRight + deltaLeft) / 2) * sin(robotPose(3));

    % Vérification des collisions avec la carte d'occupation
    robotPose(1) = robotPose(1) + deltaX;
    robotPose(2) = robotPose(2) + deltaY;
    robotPose(3) = robotPose(3) + deltaTheta;

    % Affichage en temps réel
    show(myOccMap);
    hold on;
    xlim([-4 4]); 
    ylim([-4 4]); 
    % Afficher la pose estimée par AMCL avec un modèle 3D
    frameSize = wheelBase / 0.95; % Ajuster selon la taille réelle du robot

    % Définir la position et l'orientation estimées
    plotTrVec = [robotPose(1:2); 0]; % Convertir en vecteur colonne avant concaténation
    plotRot = axang2quat([0 0 1 robotPose(3)]); % Quaternion pour la rotation

    % Tracer le repère de la pose estimée avec un modèle 3D
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", ...
                   "Parent", gca, "View", "2D", "FrameSize", frameSize);
    light; % Ajouter un effet de lumière pour améliorer la visibilité
    hold off;

    % Pause pour permettre la lecture des touches à intervalles réguliers
    pause(0.01);
    clf(1)
end
% Nettoyage à la fin de la simulation
wb_robot_cleanup();

