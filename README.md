La maxi liste des ressources pour faire (un bon) robot à Eurobot
==========================================================================================

Ce repo est destiné à compiler et partager les ressources (cours/vidéos/composants/discussions/liens) en lien avec la compétition Eurobot. Les cours, en PDF de préférence, sont sauvegardés sur le repo afin d'assurer leur longévité.

Les ressources trop générales qui sont faciles à trouver sur le net (cours pour débutant en programmation, etc) n'ont pas grand intérêt à être ici, on essaye de se concentrer sur des choses très concrètes pour Eurobot.

Voici les comités d'organisation nationaux:
* [Eurobot](https://www.eurobot.org/)
* [Coupe de France de Robotique](https://www.coupederobotique.fr/)
* [Eurobot Suisse](https://www.robot-ch.org/swisseurobot/)
* [Robotix's Belgique](https://sparkoh.be/projet-robotixs/robotixs/)
* [Eurobot Russie](https://eurobot-russia.org/)
* [Algérie, Allemagne, Roumanie, Serbie, Espagne, Tunisie, Royaume-Uni](https://www.eurobot.org/contacts/) (Ces pays ne semblent pas avoir de sites web à jour)

Vous pouvez contribuer à cette maxi liste:

* en créant une [Pull Request](https://github.com/VRAC-team/la-maxi-liste-ressources-eurobot/pulls): vous vous débrouillez pour mettre la ressource au bon endroit et avec la mise en page qui va bien
* en créant une [Issue](https://github.com/VRAC-team/la-maxi-liste-ressources-eurobot/issues): on se charge de tout mettre en page au bon endroit

**Merci aux équipes pour ces documents et aux [contributeurs de cette liste](https://github.com/VRAC-team/la-maxi-liste-ressources-eurobot/graphs/contributors) !**

(Si un document vous appartient et que vous souhaitez le faire retirer, merci de nous contacter.)



------------------------------------------------------------------------------------------
# Sommaire

```
1. Introduction
  1.1 Prérequis
  1.2 Première participation à la coupe
  1.3 Glossaire

2. Ressources académiques spécifiques à Eurobot
    2.1 Pour bien commencer
    2.2 Base roulante
        2.2.1 Odométrie
        2.2.2 Roues
        2.2.3 Moteurs
    2.3 Asservissement
    2.4 Balises
    2.5 Simulation
    2.6 Planificateur de trajectoire et évitement
    2.7 Intelligence artificielle
    2.8 Communication sans-fil
    2.9 Architecture des robots
    2.10 Code source des équipes

3. Microcontrôleurs, ordinateur à carte unique, IDE et outils
  3.1 Microcontrôleurs
    3.1.1 Arduino
    3.2.2 STM32
    3.3.3 Teensy
    3.4.4 ESP32
    3.4.5 Autres microcontrôleurs
  3.2 FPGA
  3.3 Ordinateur à carte unique
  3.4 Outils pour le développement

4. Actionneurs, capteurs, connectique
    4.1 Moteurs et controleurs
        4.1.1 Moteurs à courant continu
        4.1.2 Moteurs brushless
        4.1.3 Moteurs pas-à-pas
        4.1.4 Servomoteurs
    4.2 Roues
    4.4 Encodeurs
    4.5 Capteurs de distance
      4.5.1 Capteurs de distance à ultrasons
      4.5.2 Télemètres laser
      4.5.3 Capteurs temps de vol
      4.5.4 Capteurs photoélectriques
      4.5.5 LiDAR
    4.6 Ventouses, pompes à vide, tubes
    4.7 Camera pour traitement vidéo
    4.8 Connectique et câblage

5. Sites internet marchands et services
  5.1 Modélisme
  5.2 Mécanique
  5.3 Electronique
  5.4 Mix électronique/mécanique
  5.5 Circuit imprimé
  5.6 Usinage
  5.7 Visserie

6. Logiciels de CAO
  6.1 CAO mécanique
  6.2 CAO électronique

7. Liens en vrac
```



------------------------------------------------------------------------------------------
# 1. Introduction

Avant de commencer dans le vif du sujet, n'hésitez pas à rejoindre le discord qui est ouvert pour tous les participants de la coupe: [Eurobot - CDR](https://discord.gg/tteC3Cp).
Beaucoup d'équipes sont là pour papoter robotique, vous donner des conseils, partager leurs dernière trouvailles mais aussi pour s'amuser sur des jeux.

Le [Forum Planete Science](https://www.planete-sciences.org/forums/) n'est plus très actif depuis l'ère Discord, cependant il est toujours utilisé pour poser des questions directement aux arbitres et pour partager des "comptes rendu" d'une saison par les équipes.

## 1.1 Prérequis

Si c'est votre première participation à la coupe et que vous arrivez ici, ne vous inquietez pas au vu de la longueur de ce document, il n'y a pas besoin de tout lire pour commencer à faire des robots ^^

On part du principe que vous avez au moins les bases en électronique et en programmation.
Il n'y a pas de cours pour débutant ici, ce sont des ressources faciles à trouver sur le net.

## 1.2 Première participation à la coupe

Le conseil le plus important qu'on peut donner à une équipe qui se lance est qu'il faut en priorité avoir une base roulante fiable. C'est vraiment la fondation des robots, sans ça c'est vraiment difficile de faire des actionneurs qui peuvent marquer des points.
La motorisation la plus simple à mettre en place est très surement un robot à roues différentielles avec des moteurs pas-à-pas, elle est très rapide à mettre en place car elle peut fonctionner sans asservissement ni d'odométrie.

Le second conseil est qu'il vaut mieux avoir un seul robot bien terminé et bien rôdé plutôt que 2 robots finis trop tardivement.
On peut également faire cette remarque avec un seul actionneur simple et répétable, à la place d'un système trop complexe et qui sera difficile à mettre au point.

Le temps est souvent la plus grosse contrainte qui rencontrent les équipes. Assembler son robot quelques jours avant la coupe ne permettera pas de les finaliser correctement, il faut prévoir au moins un bon mois de test.

[Les dix commandements version Omybot](https://twitter.com/TeamOmybot/status/1128554678101467136) est une version plus concrète qui s'applique surtout lors de la coupe.

Enfin le dernier conseil c'est de regarder ce que font les autres équipes, c'est motivant et sa peut donner de nouvelles idées:
* [Chaine Youtube Planete Science](https://www.youtube.com/c/PlaneteSciencesNational/videos): on y trouve tout les matchs de la coupe de France
* [Portail des sites web des équipes par PM-ROBOTIX](https://www.pm-robotix.eu/sites-de-la-coupe-et-des-equipes/)

## 1.3 Glossaire

**Asservissement**: Système dont le but est d'atteindre le plus rapidement possible sa valeur de consigne et de la maintenir, quelles que soient les perturbations externes.

**Billes porteuses / Billes folles**: Permettent de faire un point de contact avec le sol. Principalement utilisés aux 4 extrémitées des robots à roues différentielles.

**LiDAR**: Télémètre laser à balayage, il permet de récupérer des un nombre de points sur un plan. Il est utilisé pour la détection des mats de balise et donc pour récupérer la position des concurrents. Il peut également être utlisé au niveau du sol pour récupérer la position des éléments de jeu ou mieux détecter le gabarit des robots concurrents.

**Odométrie**: Technique d'utilisation des données de capteurs permettant d'estimer la position du robot. Utilise généralement les 2 roues codeuses pour estimer la position mais peut également être faite avec un lidar.

**Roue holonome**: Roue constituée de galets répartis sur sa périphérie, elles peuvent donc se déplacer dans toutes les directions.

**Roues codeuses / Roues odométriques**: Roues souvent monté sur un pivot ou une glissière afin de toujours maintenir un contact avec le sol. Ces roues sont équipées d'encodeurs qui permettent une mesure de rotation pour le calcul de l'odométrie.



------------------------------------------------------------------------------------------
# 2. Ressources académiques spécifiques à Eurobot

## 2.1 Pour bien commencer

**RCVA: Réflexions sur un robot Eurobot en 9 chapitres**

> 1. Rappels cinématiques et définition du coefficient d’adhérence
> 2. Etude du phénomène de glissement en phase accélération
> 3. Choix d’un profil de vitesse
> 4. Les lois mécaniques et thermiques dans un moteur
> 5. Condition de glissement en cas de blocage du robot
> 6. Choix de la vitesse et de l’accélération
> 7. Essais avec enregistrements
> 8. Asservissement
> 9. Quelques questions réponses
>
> [**PDF - RCVA - Réflexions sur un robot Eurobot**](http://www.rcva.fr/wp-content/uploads/2016/12/devoir_de_vacances.pdf)

**Cubot: Asservissement polaire en 6 chapitres**

> 1. Cas de charge
> 2. Odométrie
> 3. Calibration de l'odométrie
> 4. Calcul de la consigne
> 5. Calcul des rampes de vitesse
> 6. PID
> 7. Synthèse
>
> [**PDF - Atelier asservissement polaire**](asservissement/Cubot-atelier_asservissement.pdf)
>
> [EXCEL - Simulation asservissement polaire](asservissement/Cubot-asservissement.xlsx)
>
> [EXCEL - Génération de profile trapézoidal de vitesse](asservissement/Cubot-profil_de_vitesse.xlsx)

## 2.2 Base roulante

### 2.2.1 Odométrie

* [WEB - CVRA - Odometry calibration](https://cvra.ch/robot-software/howto/calibrate-odometry/)
* [VIDEO - Robotic-System - Calibrage de l'odométrie](https://www.youtube.com/watch?v=X5PMFvVecXU)
* [PDF - RCVA - Odométrie avec correction centrifuge](odometrie/RCVA-odometrie.pdf)
* [PDF - RCVA - Trajectoires courbes et odométrie, De l’importance de la différence de diamètre des deux roues
codeuses](odometrie/RCVA-Conseils_theoriques_pour_Eurobot.pdf)
* [VIDEO - RCVA - comparaison approximation linéaire/circulaire, correction centrifuge](https://www.youtube.com/watch?v=KQzfMAJyvB0)
* [VIDEO - RCVA - odométrie](https://www.youtube.com/watch?v=557l7JOs35E)

### 2.2.2 Roues

* [**WEB - Erich Styger - Making Perfect Sticky DIY Sumo Robot Tires**](https://mcuoneclipse.com/2017/12/28/making-perfect-sticky-diy-sumo-robot-tires/)
* [FORUM - Robotech Legends - Moulage de pneus en polyuréthane](https://www.planete-sciences.org/forums/viewtopic.php?t=18632)
* [VIDEO - Barbatronic - Moulage de pneus en silicone](https://www.youtube.com/watch?v=EGPVa1ZnXe8)
* [VIDEO - Micro Technology - test d'adhérence des roues](https://www.youtube.com/watch?v=cPfP7zyS0kU)

### 2.2.3 Moteurs

* [**WEB - RobotShop - Outil de Dimensionnement d'un moteur d'entrainement**](https://www.robotshop.com/community/blog/show/dimensionnement-dun-moteur-dentranement)
* [PDF - ANCR - Dimensionner ses moteurs](moteurs/ANCR-Dimensionner_ses_moteurs.pdf)
* [PDF - TechTheTroll - Dimensionnement des moteurs de propulsion](https://techthetroll.files.wordpress.com/2016/06/techthetroll-dimensionnement-des-moteurs-de-propulsion.pdf)

## 2.3 Asservissement

* [PDF - totofweb - Le PID utilisé en régulation de position et/ou de vitesse de moteurs électriques](asservissement/totofweb-PID_régulation_de_position_vitesse.pdf)
* [PDF - Microb Technology - Documentation de l'asservissement, librairie Aversive, évitement](asservissement/MicrobTechnology-wiki_asservissement_lib_aversive.pdf)
* [PDF - RCVA - Asservissement du robot à une trajectoire](http://www.rcva.fr/wp-content/uploads/2016/12/asservissement.pdf)
* [PDF - RCVA - Montée de tremplin par le robot RCVA sans terme intégral](http://www.rcva.fr/wp-content/uploads/2016/12/Montee_De_Tremplin_Sans_Terme_Integral_RCVA.pdf)
* [VIDEO - RCVA - Asservissement en rotation avec un gyromètre ADXRS453](https://www.youtube.com/watch?v=p0cm7LnMXOc)
* [VIDEO - RCVA - cours asservissement polaire](https://www.youtube.com/watch?v=JYZ_2y8k1Os)
* [PDF - TechTheTroll - Les trajectoires courbes dans la bonne humeur: de l’asservissement à la planification](https://techthetroll.files.wordpress.com/2016/07/trajectoire_courbe.pdf)
* [WEB - Implémenter un PID sans faire de calculs ! - Ferdinand Piette](http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/)

## 2.4 Balises

* [PDF - totofweb - Balise infrarouge](balise/totofweb-balises_IR.pdf)
* [WEB - Barbatronic - Reflective lidar for robotic and the eurobot competition](http://fabacademy.org/2019/labs/lamachinerie/students/adrien-bracq/projects/final-project/)
* [PDF - CVRA - Development of an ultra-wide band indoor positioning system](https://github.com/cvra/robot-software/blob/1d208d0d5882d5526eef758eae61f5626291a016/uwb-beacon-firmware/doc/report.pdf)
* [PDF - CVRA - Balises laser Eurobot 2008](https://cvra.ch/ressources/misc/balise_laser.pdf)
* [PDF - Microb Technology - Faire des balises laser en buvant des bières](balise/MicrobTechnology-Faire_des_balises_laser_en_buvant_des_bieres.pdf)
* [VIDEO - ESEO - localisation par balises infrarouges](https://www.youtube.com/watch?v=bGoXEwQ0UQs)

## 2.5 Simulation

* [VIDEO - ESEO - simulateur de match](https://www.youtube.com/watch?v=fo-87AF2Fr4), [article sur leur site](https://robot-eseo.fr/strategie-du-robot-sur-simulateur/)

## 2.6 Planificateur de trajectoire et évitement

* [LIBRAIRIE- The Kraken Pathfinding - A tentacle-based pathfinding library for nonholonomic robotic vehicles](https://github.com/kraken-robotics/The-Kraken-Pathfinding)
* [LIBRAIRIE - PythonRobotics: Python sample codes for robotics algorithms](https://atsushisakai.github.io/PythonRobotics/)

## 2.7 Intelligence artificielle

* [VIDEO - @nesnes - Coder une IA pour Eurobot](https://www.youtube.com/channel/UCg1vR097bAzmJzeMBWl7Zzw), [depot GitHub Eurobot-AI](https://github.com/nesnes/Eurobot-AI)

## 2.8 Communication sans-fil

* [FORUM - Pourquoi éviter le WiFi 2.4GHz](https://www.planete-sciences.org/forums/viewtopic.php?f=97&t=16969)

## 2.9 Architecture des robots

* [WEB - Robotech Legends 2019](https://twitter.com/robotech34/status/1129147494859005952)
* [Librarie diagrams.net par @kmikaz51](https://drive.google.com/file/d/1W76g7xKKJeluGIWmfG8v30gIhi9cbWxQ/view?usp=sharing) pour déssiner votre propre architecture.

## 2.10 Code source des équipes

* [APB Team](http://git.ni.fr.eu.org/apbteam.git/tree/)
* [ARIG](https://github.com/ARIG-Robotique)
* [CVRA](https://github.com/cvra)
* [ESEO](https://github.com/ClubRobotEseo)
* [EsialRobotik](https://github.com/EsialRobotik)
* [GRUM](https://gitlab.com/grumoncton)
* [Les Karibous](https://github.com/LesKaribous)
* [Microb Technology](https://github.com/onitake/aversive)
* [UTCoupe](https://github.com/utcoupe)



------------------------------------------------------------------------------------------
# 3. Microcontrôleurs, ordinateur à carte unique et leurs IDE

## 3.1 Microcontrôleurs

### 3.1.1 Arduino

[Arduino](https://www.arduino.cc/)

IDE:
* [Arduino IDE](https://www.arduino.cc/en/software)
* [Visual Studio Code](https://code.visualstudio.com/) + [extension Arduino](https://marketplace.visualstudio.com/items?itemName=vsciot-vscode.vscode-arduino)
* VSCode/Atom/CLion/Eclipse/SublimeText/Emacs/Vim + [extension PlatformIO](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)

### 3.2.2 STM32

* [STM32](https://www.st.com/en/evaluation-tools/stm32-mcu-mpu-eval-tools.html)
* [Cartes de développement Nucleo](https://www.st.com/en/evaluation-tools/stm32-nucleo-boards.html)

Librairies:
* [LL, HAL, CMSIS](https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html#products)
* [Mbed OS](https://os.mbed.com/code/): open-source operating system for platforms using Arm microcontrollers
* [Arduino core for STM32](https://github.com/stm32duino/Arduino_Core_STM32)
* [libopencm3](https://github.com/libopencm3/libopencm3): open-source firmware library for various ARM Cortex-M microcontrollers
* [ChibiOS](https://github.com/ChibiOS/ChibiOS): complete development environment for embedded applications including RTOS, an HAL, peripheral drivers, support files and tools.

IDE:
* VSCode/Atom/CLion/Eclipse/SublimeText/Emacs/Vim + [extension PlatformIO](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)
* [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
* [Mbed online compiler](https://os.mbed.com/): IDE en ligne, pas de débogueur, intègre un gestionnaire de version. Attention cependant à avoir une solution de secours lors de la coupe au cas où il serait en maintenance quelques heures
* [Mbed Studio](https://os.mbed.com/studio/)

### 3.3.3 Teensy

[Teensy](https://www.pjrc.com/teensy/)

IDE:
* [Arduino IDE](https://www.arduino.cc/en/software) + [extension Teensyduino](https://www.pjrc.com/teensy/teensyduino.html)
* VSCode/Atom/CLion/Eclipse/SublimeText/Emacs/Vim + [extension PlatformIO](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)

### 3.4.4 ESP32

[Cartes de développement ESPRESSIF](https://www.espressif.com/en/products/devkits)

librairies:
* [Arduino core for ESP32](https://github.com/espressif/arduino-esp32)
* [ESP-IDF](https://www.espressif.com/en/products/sdks/esp-idf)

IDE:
* [Arduino IDE](https://www.arduino.cc/en/software)
* VSCode/Atom/CLion/Eclipse/SublimeText/Emacs/Vim + [extension PlatformIO](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)

### 3.4.5 Autres microcontrôleurs

* [Cypress PSoC 5](https://www.cypress.com/documentation/development-kitsboards/cy8ckit-059-psoc-5lp-prototyping-kit-onboard-programmer-and), [Cypress PSoC 6](https://www.cypress.com/documentation/development-kitsboards/psoc-6-ble-prototyping-kit-cy8cproto-063-ble)
* [WEB - The amazing $1 microcontroller by Jay Carlson](https://jaycarlson.net/microcontrollers/)

## 3.2 FPGA

* [Terasic DE0-Nano](https://www.terasic.com.tw/cgi-bin/page/archive.pl?No=593)
* [TinyFPGA](https://tinyfpga.com/)
* [WEB - Cheap FPGA Development Boards by Joel W.](https://joelw.id.au/FPGA/CheapFPGADevelopmentBoards)

## 3.3 Ordinateur à carte unique

* [RaspberryPi](https://www.raspberrypi.org/)
* [BeagleBone](https://beagleboard.org/)
* [NVidia Jetson](https://www.nvidia.com/fr-fr/autonomous-machines/jetson-store/)
* [LattePanda](https://www.lattepanda.com/)

## 3.4 Outils pour le développement

Visualisation de données:
* [Serial Port Plotter](https://github.com/CieNTi/serial_port_plotter): windows application that displays real time data from serial port, built with Qt
* [PlotJuggler](https://github.com/facontidavide/PlotJuggler): affichage de données, intégration avec ROS
* [extension Teleplot pour VSCode](https://marketplace.visualstudio.com/items?itemName=alexnesnes.teleplot): par @nesnes. Plots telemetry sent over Serial or UDP Packets.

Terminaux:
* [ScriptCommunicator](https://github.com/szieke/ScriptCommunicator_serial-terminal): Terminal multiplateforme scriptable pour port série, UDP/TCP, SPI, I2C et CAN
* [YAT: Yet Another Terminal](https://sourceforge.net/projects/y-a-terminal/): Terminal pour Windows RS-232/422/423/485, UDP/TCP, USB Serial
* [MobaXterm](https://mobaxterm.mobatek.net/): Terminal amélioré pour Windows , client SSH à onglets, outils réseau, ..



------------------------------------------------------------------------------------------
# 4. Actionneurs, capteurs, connectique

## 4.1 Moteurs et controleurs

### 4.1.1 Moteurs à courant continu

* [Maxon Motor](https://www.maxongroup.com/)
* [Faulhaber](https://www.faulhaber.com/)

### 4.1.2 Moteurs brushless

Controleurs de moteurs brushless:
* [SimpleFOC](https://simplefoc.com/): Arduino Compatible Open Source Field Oriented Control (FOC)
* [oDrive](https://odriverobotics.com/)
* [MJBOTS moteus](https://mjbots.com/products/moteus-r4-8)
* [Flipsky ODESC](https://flipsky.net/collections/electronic-products/products/odesc3-6-optimization-of-high-performance-brushless-motor-high-power-driver-foc-bldc-based-on-odrive)

### 4.1.3 Moteurs pas-à-pas

* [OMC-StepperOnline](https://www.omc-stepperonline.com/)
* [Trinamic](https://www.trinamic.com/)

### 4.1.4 Servomoteurs

servomoteurs classiques:
* [HITEC](https://hitecrcd.com/)

Servomoteurs intelligents:
* [HerkuleX DRS-0101](http://hovis.co.kr/guide/main_eng.html)
* [Dynamixel AX-12](http://www.robotis.us/ax-series/)

## 4.2 Roues

* [JSumo](https://www.jsumo.com/wheels)
* [Fingertech](https://www.fingertechrobotics.com/products.php?cat=Wheels+%26+Hubs)
* [BaneBots](http://www.banebots.com/category/T40P.html)

## 4.4 Encodeurs

* [encodeur rotatif à effet Hall AMS](https://ams.com/angle-position-on-axis)
* [encodeur rotatif capacitifs CUI](https://www.cuidevices.com/catalog/motion/rotary-encoders/incremental/modular)
* [encodeur rotatif optique Kubler](https://www.kuebler.com/fr/produits/mesure/codeurs/product-finder)
* [encodeur rotatif inductif POSIC](https://www.posic.com/EN/products/rotary-encoders.html)

## 4.5 Capteurs de distance

### 4.5.1 Capteurs de distance à ultrasons

* [SICK](https://www.sick.com/fr/en/distance-sensors/ultrasonic-sensors/um18/c/g185679): UM18 utilisé à la coupe
* [Baumer](https://www.baumer.com/de/en/product-overview/distance-measurement/ultrasonic-distance-sensors/c/290): UNAM, UNDK utilisé à la coupe
* [Pepperl+Fuchs](https://www.pepperl-fuchs.com/global/en/classid_186.htm)

### 4.5.2 Télemètres laser

* [SICK Dx35](https://www.sick.com/fr/en/distance-sensors/mid-range-distance-sensors/dx35/c/g261053)
* [Baumer](https://www.baumer.com/de/en/product-overview/distance-measurement/laser-distance-sensors/c/289)

## 4.5.3 Capteurs temps de vol

[Capteurs ToF STMicroelectronics](https://www.st.com/en/imaging-and-photonics-solutions/proximity-sensors.html#products)

### 4.5.4 Capteurs photoélectriques

* [SICK](https://www.sick.com/us/en/photoelectric-sensors/c/g172752?q=:Def_Type:Product)
* [Pepperl+Fuchs](https://www.pepperl-fuchs.com/global/en/classid_8.htm)

### 4.5.5 LiDAR

* [Slamtec RPLIDAR A2](https://www.slamtec.com/en/Lidar/A2): Utilisé par de nombreuses équipes à la coupe
* [Pepperl+Fuchs](https://www.pepperl-fuchs.com/france/fr/R2000_Detection_laser_scanner.htm): R2000 utilisé à la coupe
* [Inno-maker LD06](https://www.inno-maker.com/product-category/products/ladir/)
* [Hokuyo URG-04LX-UG01](https://www.hokuyo-aut.jp/search/single.php?serial=166)
* [SICK TiM5xx](https://www.sick.com/fr/fr/solutions-de-mesure-et-de-detection/capteurs-2d-lidar/tim5xx/c/g292754)
* [Ydlidar X4](https://www.ydlidar.com/products/view/5.html)

## 4.6 Ventouses, pompes à vide, tubes

* [Coval](https://www.coval.fr/produits/)
* [Piab](https://www.piab.com/fr-FR/Produits/ventouses/)
* [Festo](https://www.festo.com/cat/fr_fr/products)
* [KNF](https://www.knf.fr/fr/produits/pompes-oem/)
* [Thomas Gardner Denver](https://www.gardnerdenver.com/en-us/thomas/gas-pumps)
* [SCHMALZ](https://www.schmalz.com/fr/technique-du-vide-pour-l-automation/composants-pour-le-vide)

## 4.7 Camera pour traitement vidéo

* [OpenMV](https://openmv.io/)
* [JeVois](https://www.jevoisinc.com/)
* [Pixy](https://pixycam.com/)
* [Intel RealSense](https://www.intel.fr/content/www/fr/fr/architecture-and-technology/realsense-overview.html)

## 4.8 Connectique et câblage

* [Wurth Electronik](https://www.we-online.com/catalog/en/em/connectors/)
* [TE](https://www.te.com/)
* [Samtec](https://www.samtec.com/)



------------------------------------------------------------------------------------------
# 5. Sites internet marchands et services

## 5.1 Modélisme

Batteries, chargeurs, moteurs, servos, roues, ...
* [Miniplanes](https://www.miniplanes.fr/)
* [HobbyKing](https://hobbyking.com/)
* [mcmracing](https://www.mcmracing.com/)
* [EuroRC](https://www.eurorc.com/)
* [Roues Lynxmotion](https://www.robotshop.com/eu/fr/lynxmotion-roues.html)

## 5.2 Mécanique

Composants mécanique:
* [Misumi](https://fr.misumi-ec.com/): composants mécanique configurables, visserie, bruts, ..
* [Motedis](https://www.motedis.fr/): profilées aluminium 2020, bruts alu, ..
* [MakerBeam](https://www.makerbeam.com/): profilées aluminium 1010, 1515, glissières, ..
* [Systeal](https://www.systeal.com/fr/): profilées aluminium, ..
* [Technic-Achat](https://www.technic-achat.com/): profilées aluminium 2020, ..
* [123-Roulements](https://www.123roulement.com/)
* [QualiChutes](http://www.qualichutes.com/): chutes acier, aluminium, plastiques, ...

Plastiques:
* [Lacrylicshop](https://www.decoupe-plexi-sur-mesure.com/)
* [Plexiglas-shop](https://www.plexiglas-shop.com/)
* [SuperPlastic](https://www.superplastic.be/)
* [Pyrasied](https://www.pyrasied.nl/)

## 5.3 Electronique

Composants électroniques:
* [Mouser](https://www.mouser.fr/)
* [Farnell](https://fr.farnell.com/)
* [Digi-Key](https://www.digikey.fr/)
* [TME](https://www.tme.eu/fr/)
* [Arrow](https://www.arrow.com/fr-fr)
* [rutronik24](https://www.rutronik24.com/)

## 5.4 Mix électronique/mécanique

Fabricants:
* [Pololu](https://www.pololu.com/): contrôleur de moteurs dc/brushless/stepper, capteurs, cartes de dev, ...
* [Sparkfun](https://www.sparkfun.com/)
* [Adafruit](https://www.adafruit.com/)

Distributeurs:
* [Robot Maker](https://www.robot-maker.com/shop/): 10% de réduction pour les associations qui participent à la coupe de France
* [GoTronic](https://www.gotronic.fr/)
* [Lextronic](https://www.lextronic.fr/)
* [RS Components](https://fr.rs-online.com/)
* [RobotShop](https://www.robotshop.com/)
* [Conrad](https://www.conrad.fr/)
* [Distrelec](https://www.distrelec.fr/)
* [Antratek](https://www.antratek.com/)
* [Watterott](https://shop.watterott.com/)
* [goBILDA](https://www.gobilda.com/)

## 5.5 Circuit imprimé

* [Aisler](https://aisler.net/)
* [Eurocircuits](https://www.eurocircuits.com/)
* [OSHPark](https://oshpark.com/)
* [PCBWay](https://www.pcbway.com/)
* [Seeed Studio](https://www.seeedstudio.com/fusion_pcb.html)
* [JLCPCB](https://jlcpcb.com/)
* Comparateur de prix: [PCBShopper](https://pcbshopper.com/)

## 5.6 Usinage

Découpe laser, impréssion 3D, fraisage, tournage, ...
* [JohnSteel](https://www.john-steel.com/fr/)
* [Usineur.fr](https://www.usineur.fr/)
* [Protolabs](https://www.protolabs.fr/)
* [Xmake](https://www.xmake.com/)
* [Usinage boîtier](https://www.frontpanelexpress.com/)

## 5.7 Visserie

* [Bossard](https://eu.shop.bossard.com/fr/fr/)
* [Cergy-Vis](https://www.cergy-vis.fr/)
* [Vis Express](https://www.vis-express.fr/en/)
* [Bricovis](https://www.bricovis.fr/)
* [Technifix](http://www.technifix.be/)



------------------------------------------------------------------------------------------
# 6. Logiciels de CAO

## 6.1 CAO mécanique

* [KiCad](https://kicad-pcb.org/)
* [Altium Designer](https://www.altium.com/altium-designer/)
* [EasyEDA](https://easyeda.com/fr): Outil en ligne
* [Eagle](https://www.autodesk.fr/products/eagle/free-download)
* [LibrePCB](https://librepcb.org/)
* [Alegro PCB Designer](https://www.cadence.com/en_US/home/tools/pcb-design-and-analysis/pcb-layout/allegro-pcb-designer.html)

Outils:
* [PCB Panelizer & Gerber tool suite](http://blog.thisisnotrocketscience.nl/projects/pcb-panelizer/)
* [PCB CheckList](https://github.com/azonenberg/pcb-checklist): Une checklist qui permet de vérifier si on à pas fait une erreur lors de la conception

## 6.2 CAO électronique

* [Fusion 360](https://www.autodesk.com/products/fusion-360/overview)
* [SolidWorks](https://www.solidworks.com/)
* [FreeCAD](https://www.freecadweb.org/)
* [OpenSCAD](http://www.openscad.org/): The Programmers Solid 3D CAD Modeller
* [SolveSpace](http://solvespace.com/): parametric 2d/3d CAD



------------------------------------------------------------------------------------------
# 7. Liens en vrac

Forums robotique:
* [Forum Planete Science / Coupe de France de robotique](https://www.planete-sciences.org/forums/)
* [Forum Robot Maker](https://www.robot-maker.com/forum/)
* [Forum Usinages.com (catégorie Robotique et Domotique)](https://www.usinages.com/forums/robotique-et-domotique.125/)

Equipes Eurobot (non listés sur le portail des équipes PM-ROBOTIX):
* [Roboterclub Aachen](https://www.roboterclub.rwth-aachen.de/)

Electronique:
* [WEB - Texas Instruments - Design tools & simulation](https://www.ti.com/design-resources/design-tools-simulation.html): conception de filtres, architecture d'arbres d'horloges, .. 
* [LS7366R 32-bit quadrature counter with serial interface](https://lsicsi.com/datasheets/LS7366R.pdf)
* [Texas Instrument LM628/LM629](http://www.ti.com/lit/ds/symlink/lm629.pdf): CI dédié l'asservissement d'un moteur, commande en position/vitesse/accélération

Mécanique:
* [Générateur en ligne de roue à courroie, roues dentées, crémaillères](https://www.igus.fr/info/3d-print-gears): fichiers de CAO téléchargeables pour les imprimer en 3D ou les commander
* [Générateur de courroie GT2 imprimable en 3D](https://www.thingiverse.com/thing:3458902) avec Fusion 360
* [Système de guidage linéaire motorisé](https://gitlab.cba.mit.edu/jakeread/pgd), conçu pour être fabriqué avec de la découpe laser et de l'impression 3D

Logiciel:
* [PyRobot - light weight, high-level interface which provides hardware independent APIs for robotic manipulation and navigation by facebook research.](https://pyrobot.org/)

Littérature:
* [Elements of Robotics, Mordechai Ben-Ari, Francesco Mondada, 2018, Open Access](https://www.springer.com/gp/book/9783319625324)
* [PDF - Rich LeGrand - Closed-Loop Motion Control for Mobile Robotics](https://www.cs.hmc.edu/~dodds/projects/RobS05/XPort/XPortArticle.pdf): un Game Boy Advance, des roues holonomes, des legos et hop une base mobile
* [La RACHE, une méthodologie réaliste mais formaliste - par Sukender](https://www.la-rache.com/img/a1_rache.pdf), [Système d'unités pifométriques](https://www.la-rache.com/img/unites.pdf)
* [PDF - High-Precision Robot Odometry Using an Array of Optical Mice](https://pdfs.semanticscholar.org/37ca/1fc2dcc4c0cf860fc0b00542fc7cb59c579f.pdf)
