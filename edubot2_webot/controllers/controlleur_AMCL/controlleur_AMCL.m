load('scans_test.mat','myOccMap')
% Contrôleur MATLAB pour le robot Eduro dans Webots
wb_robot_init();

% Temps de simulation
TIME_STEP = 33;

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
wheelBase = 0.378+0.034; % Distance entre les roues en mètres (0.2 * 2)

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

% Initialiser AMCL
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.4 0.4 0.4 0.4];

rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.Map = myOccMap;
rangeFinderModel.SensorLimits = [0.1 10];

amcl = monteCarloLocalization;
amcl.UseLidarScan = true;
amcl.ParticleLimits = [500 5000];
amcl.GlobalLocalization = false;
amcl.UpdateThresholds = [0.01, 0.01, 0.01];
amcl.ResamplingInterval = 1;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

amcl.InitialPose = [webots_initial_y; -webots_initial_x; -webots_initial_theta ];
amcl.InitialCovariance = diag([0.001, 0.001, 0.00001]);

figure(1)
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

    %     % Récupérer la position initiale
    position = wb_supervisor_node_get_position(robot_node);
    orientation = wb_supervisor_node_get_orientation(robot_node);
        % Extraire les coordonnées X, Y, et l'angle theta
    webots_x = position(1); % Position X
    webots_y = position(2); % Position y sur webots
    webots_theta = atan2(orientation(2,1), orientation(1,1)) + pi/2 ;
    
    % Récupérer les données du LiDAR
    lidarRanges = wb_lidar_get_range_image(lidar);
    
    % Vérifier si les données sont valides
    if ~isempty(lidarRanges)
        % Convertir les données en coordonnées cartésiennes (x, y)
        cart = zeros(lidarResolution, 2);
        for i = 1:lidarResolution
            r = lidarRanges(i);  % Distance mesurée
            if r > 0  % Filtrer les valeurs invalides
                cart(i, :) = [r * cos(lidarAngles(i))-0.1395, -r * sin(lidarAngles(i))];
            end
        end

        % Vérifier si `cart` contient des points valides avant d'enregistrer
        if any(cart(:, 1) ~= 0)
            scan = lidarScan(cart);  % Créer l'objet `lidarScan`
        end
    end
    
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

    robotPose(1) = robotPose(1) + deltaX;
    robotPose(2) = robotPose(2) + deltaY;
    robotPose(3) = robotPose(3) + deltaTheta;

    % mise a jour amcl
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(robotPose, scan);

    % Afficher la différence
    Diff_x = webots_x + estimatedPose(2);
    diff_y = webots_y - estimatedPose(1);
    diff_theta = webots_theta + estimatedPose(3);
    % Normaliser l'angle entre -pi et pi
    diff_theta = mod(diff_theta + pi, 2*pi) - pi;

    diff = [Diff_x, diff_y, diff_theta];
    
    disp(['erreur : ', num2str(diff)]);
    
    % Affichage en temps réel
    show(myOccMap);
    hold on;
    xlim([-4 4]); 
    ylim([-4 4]); 
    % Afficher la pose odométrique actuelle (en vert)
    plot(webots_y, -webots_x, 'go', 'MarkerSize', 4, 'LineWidth', 3);

    % Afficher la pose estimée par AMCL avec un modèle 3D
    frameSize = wheelBase / 0.95; % Ajuster selon la taille réelle du robot
    
    % Définir la position et l'orientation estimées
    plotTrVec = [estimatedPose(1:2).'; 0]; % Convertir en vecteur colonne avant concaténation
    plotRot = axang2quat([0 0 1 estimatedPose(3)]); % Quaternion pour la rotation
    
    % Tracer le repère de la pose estimée avec un modèle 3D
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", ...
                   "Parent", gca, "View", "2D", "FrameSize", frameSize);
    light; % Ajouter un effet de lumière pour améliorer la visibilité


    % Tracer les particules AMCL (en bleu clair)
    particles = getParticles(amcl);
    plot(particles(:,1), particles(:,2), 'b.', 'MarkerSize', 2);
    
    % Afficher les scans LiDAR sur la carte (en rouge)
    if ~isempty(scan)
        transformedScan = transformScan(scan, estimatedPose); % Transformer par rapport à la pose estimée
        plot(transformedScan.Cartesian(:,1), transformedScan.Cartesian(:,2), 'r.', 'MarkerSize', 1);
    end
    hold off;

    pause(0.01);
    %clf(1)
end

% Nettoyage à la fin de la simulation
wb_robot_cleanup();
