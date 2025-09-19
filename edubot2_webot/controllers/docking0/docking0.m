load('refScanDocking.mat', 'refScan');  % refScan est un lidarScan
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
% refScan = removeInvalidData(refScan);

% initialisation PID Docking
% PID pour l’orientation
Kp_theta = 1.5;
Ki_theta = 0;
Kd_theta = 0.05;
int_theta = 0;
prev_theta_error = 0;
% PID pour la distance
Kp_dist = 0.8;
Ki_dist = 0;
Kd_dist = 0.05;
int_dist = 0;
prev_dist_error = 0;
% État du docking
dockingMode = false;

% Activer le clavier
wb_keyboard_enable(TIME_STEP);
maxSpeed = 3;% rad/s

figure(2)
% Boucle principale de la simulation
while wb_robot_step(TIME_STEP) ~= -1


    % Lire la touche pressée
    Key = wb_keyboard_get_key();
    % Vérifier si la touche 'Q' est pressée
    if Key == 'Q' || Key == 'q'
        break;
    end
    % Initialiser les vitesses des moteurs
    leftSpeed = 0;
    rightSpeed = 0;
    if ~dockingMode   
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
    end

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

    % Récupérer les données du LiDAR
    lidarRanges = wb_lidar_get_range_image(lidar);

    if ~isempty(lidarRanges)
        cart = zeros(lidarResolution, 2);
        for i = 1:lidarResolution
            r = lidarRanges(i);
            if r > 0
                cart(i, :) = [r * cos(lidarAngles(i)) - 0.1395, -r * sin(lidarAngles(i))];
            end
        end
    
        % Filtrage des points au-delà de 3m
        maxRange = 3;
        distances = sqrt(cart(:,1).^2 + cart(:,2).^2);
        cartFiltered = cart(distances <= maxRange, :);
        
        if size(cartFiltered, 1) >= 10
            currentScan = lidarScan(cartFiltered);
            % if exist('currentScan', 'var')
            %     refScan = currentScan;
            %     save('refScanDocking.mat', 'refScan');
            %     disp(" Scan de la station de docking enregistré !");
            % end
            if exist('refScan', 'var') && isa(currentScan, 'lidarScan')
                try
                    initialRelPose = [0 0 0]; % ou une estimation approximative
                    [transform, stats] = matchScansLine(currentScan, refScan, initialRelPose, ...
                    'CompatibilityScale', 0.003);
                    transform(1:2) = -transform(1:2);  % inversion position
                    transform(3) = -transform(3);      % inversion angle

                    % Activer le mode docking après détection valide
                    if ~dockingMode && all(~isnan(transform))
                        dockingMode = true;
                        disp(' Mode docking ACTIVÉ !');
                    else
                        dockingMode = false;
                    end
                catch ME
                    disp([' Erreur matchScansLine : ', ME.message]);
                end              
                if  dockingMode 
                    % Erreurs
                    xError = transform(1);  % distance avant
                    yError = transform(2);  % distance latérale
                    % theta proportionnelle à yError 
                    % Pondération ajustable (plus c’est grand, plus il corrigera tôt)
                    lambda = 0.6;
                    thetaError = transform(3) + lambda * atan2(yError, xError);

                
                    % PID orientation
                    int_theta = int_theta + thetaError * TIME_STEP/1000;
                    der_theta = (thetaError - prev_theta_error) / (TIME_STEP/1000);
                    w = Kp_theta*thetaError + Ki_theta*int_theta + Kd_theta*der_theta;
                    prev_theta_error = thetaError;
                
                    % PID avance (sur x uniquement si on est bien orienté)
                    if abs(thetaError) < 0.2  % éviter d'avancer si trop mal orienté
                        int_dist = int_dist + xError * TIME_STEP/1000;
                        der_dist = (xError - prev_dist_error) / (TIME_STEP/1000);
                        v = Kp_dist*xError + Ki_dist*int_dist + Kd_dist*der_dist;
                        prev_dist_error = xError;
                    else
                        v = 0;
                    end
                
                    % Calcul des vitesses moteurs
                    v_left = v - w * wheelBase/2;
                    v_right = v + w * wheelBase/2;              
                    % Conversion vitesse linéaire -> rad/s
                    leftSpeed = v_left / wheelRadius;
                    rightSpeed = v_right / wheelRadius;
                    % Saturation
                    leftSpeed = max(min(leftSpeed, 0.8), -0.8);
                    rightSpeed = max(min(rightSpeed, 0.8), -0.8);
                    % Appliquer la vitesse
                    wb_motor_set_velocity(leftMotor, leftSpeed);
                    wb_motor_set_velocity(rightMotor, rightSpeed);
                    fprintf("Erreur X: %.3f | Y: %.3f | theta: %.3f\n", xError, yError, thetaError);
                    % Arrêt si on est arrivé
                    if abs(xError) < 0.025 && abs(yError) < 0.02 && abs(thetaError) < 0.025
                        dockingMode = false;
                        wb_motor_set_velocity(leftMotor, 0);
                        wb_motor_set_velocity(rightMotor, 0);
                        disp('Docking TERMINÉ ! Robot aligné.');
                    end
                end
            end
        end
    end
    % affichage des scans
    plot(refScan)
    hold on
    plot(currentScan)
    hold off  
    
    pause(0.001);
end

% Nettoyage à la fin de la simulation
wb_robot_cleanup();
