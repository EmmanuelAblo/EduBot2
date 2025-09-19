readme

il existe pour le moments dans le dossier \controllers de notre monde webots (..code\edubot2_webot\controllers\…) trois différents contrôleurs:

 - my_controller0 : permet juste de déplacer le robots dans l'environnement et enregistrer les variables "scans" et "odoms" dans un Workspace en offline

 - odom_realtime : permet de visualiser le scan lidar et de déplacer le robots sur l'occupancy grid map en temps réel

 - controlleur_AMCL : pour realiser l'amcl


cmd Windows pour lancer un des contrôleurs webots en extern:

     *  webots-controller.exe [options] path/to/controller/file

pour lancer 'controlleur_affichage_tempsreel' par exemple :

     *	webots-controller.exe C:\Users\emman\Desktop\PFE_Mastech\code\edubot2_webot\controllers\controlleur_affichage_tempsreel\controlleur_affichage_tempsreel.m

on peut aussi le lancer en mode interactif avec :

     *	webots-controller.exe --interactive C:\Users\emman\Desktop\PFE_Mastech\code\edubot2_webot\controllers\controlleur_affichage_tempsreel\controlleur_affichage_tempsreel.m


NB: les donner clavier sont recupere par webots a present il faut donc cliquer sur l'environement webots avant de pourvoir faire deplaver le bots avec le clavier
