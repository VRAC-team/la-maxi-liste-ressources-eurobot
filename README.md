La maxi liste des ressources pour faire (un bon) robot à Eurobot
==========================================================================================

Ce repo est destiné à compiler et partager les ressources (cours/vidéos/composants/discussions/liens) en lien avec la compétition [Eurobot](https://www.eurobot.org/). Les cours, en PDF de préférence, sont sauvegardés sur le repo afin d'assurer leur longévité.

Les ressources trop générales qui sont faciles à trouver sur le net (cours pour débutant en programmation, etc) n'ont pas grand intérêt à être ici, on essaye de se concentrer sur des choses très concrètes pour Eurobot.

Vous pouvez contribuer à cette maxi liste:

* en créant une [Pull Request](https://github.com/VRAC-team/la-maxi-liste-ressources-eurobot/pulls): vous vous débrouillez pour mettre la ressource au bon endroit et avec la mise en page qui va bien
* en créant une [Issue](https://github.com/VRAC-team/la-maxi-liste-ressources-eurobot/issues): on se charge de tout mettre en page au bon endroit

**Merci aux équipes pour ces documents et aux [contributeurs de cette liste](https://github.com/VRAC-team/la-maxi-liste-ressources-eurobot/graphs/contributors) !**

(Si un document vous appartient et que vous souhaitez le faire retirer, merci de nous contacter.)




------------------------------------------------------------------------------------------
# Sommaire

- [Sommaire](#sommaire)
- [1. Introduction](#1-introduction)
   * [1.1 Prérequis](#11-prérequis)
   * [1.2 Conseils pour une première participation à la coupe](#12-conseils-pour-une-première-participation-à-la-coupe)
   * [1.3 Glossaire](#13-glossaire)
- [2. Ressources spécifiques à Eurobot](#2-ressources-spécifiques-à-eurobot)
   * [2.1 Pour bien commencer](#21-pour-bien-commencer)
   * [2.2 Odométrie](#22-odométrie)
   * [2.3 Roues](#23-roues)
   * [2.4 Moteurs](#24-moteurs)
   * [2.5 Asservissement](#25-asservissement)
   * [2.6 Robot holonome](#26-robot-holonome)
   * [2.7 Balises](#27-balises)
   * [2.8 Batteries](#28-batteries)
   * [2.9 Simulation](#29-simulation)
   * [2.10 Planificateur de trajectoire et évitement](#210-planificateur-de-trajectoire-et-évitement)
   * [2.11 Intelligence artificielle](#211-intelligence-artificielle)
   * [2.12 Communication sans-fil](#212-communication-sans-fil)
   * [2.13 Architecture des robots](#213-architecture-des-robots)
   * [2.14 Table et les elements de jeu](#214-table-et-les-elements-de-jeu)
- [3. Microcontrôleurs, ordinateur à carte unique, IDE](#3-microcontrôleurs-ordinateur-à-carte-unique-ide)
   * [3.1 Microcontrôleurs](#31-microcontrôleurs)
   * [3.2 FPGA](#32-fpga)
   * [3.3 Ordinateur à carte unique](#33-ordinateur-à-carte-unique)
   * [3.4 Outils pour le développement](#34-outils-pour-le-développement)
- [4. Logiciels de CAO](#4-logiciels-de-cao)
   * [4.1 CAO électronique](#41-cao-électronique)
   * [4.2 CAO mécanique](#42-cao-mécanique)
- [5. Actionneurs, capteurs, connectique](#5-actionneurs-capteurs-connectique)
   * [5.1 Moteurs et controleurs](#51-moteurs-et-controleurs)
   * [5.2 Roues](#52-roues)
   * [5.3 Encodeurs](#53-encodeurs)
   * [5.4 Capteurs de distance](#54-capteurs-de-distance)
   * [5.5 Ventouses, pompes à vide, tubes](#55-ventouses-pompes-à-vide-tubes)
   * [5.6 Camera pour traitement vidéo](#56-camera-pour-traitement-vidéo)
   * [5.7 Connectique et câblage](#57-connectique-et-câblage)
- [6. Sites internet marchands et services](#6-sites-internet-marchands-et-services)
   * [6.1 Mécanique](#61-mécanique)
   * [6.2 Electronique](#62-electronique)
   * [6.3 Mix électronique/mécanique](#63-mix-électroniquemécanique)
   * [6.4 Circuit imprimé](#64-circuit-imprimé)
   * [6.5 Modélisme](#65-modélisme)
   * [6.6 Usinage](#66-usinage)
   * [6.7 Visserie](#67-visserie)
- [7. Liens](#7-liens)
   * [7.1 Code source des équipes](#71-code-source-des-équipes)
   * [7.2 Chaines youtube des équipes](#72-chaines-youtube-des-équipes)
   * [7.3 Divers](#73-divers)




------------------------------------------------------------------------------------------
# 1. Introduction

Avant de commencer dans le vif du sujet, n'hésitez pas à **rejoindre le discord** qui est ouvert pour tous les participants de la coupe: [Eurobot - CDR](https://discord.gg/tteC3Cp).
Beaucoup d'équipes sont là pour papoter robotique, vous donner des conseils, partager leurs dernière trouvailles mais aussi pour s'amuser sur des jeux.

## 1.1 Prérequis

Si c'est votre première participation à la coupe et que vous arrivez ici, ne vous inquietez pas au vu de la longueur de ce document, il n'y a pas besoin de connaître par coeur la moindre référence listé ici ^^

On part du principe que vous avez au moins les bases en électronique et/ou en programmation.
Il n'y a pas de cours pour débutant ici, ce sont des ressources faciles à trouver sur le net.

## 1.2 Conseils pour une première participation à la coupe

Les 3 conseils les plus importants du VRAC:
- **avoir une base roulante fiable** qui permet un positionnement correct du robot (à environ 1cm près). Une base différentielle avec 2 moteurs pas a pas est le moyen le plus simple d'avoir [une bonne base roulante](https://www.youtube.com/watch?v=xO8gWP-WfIA), il n'y a même pas besoin d'odométrie ni d'aservissement!
- étudier le règlement pour faire juste 2-3 actions facile avec un actionneur simple mais qui rapportent des points à coup sur. Cela permet **d'éliminer les actions trop complexes** qui sont plus destinée aux équipes confirmés qui ont de l'expérience.
- **terminer le robot au MINIMUM 1 mois avant la compétition!**. Cette dernière phase qu'on peut penser courte va en fait réveler pleins d'imprévus/soucis qu'il faut corriger, dans l'idéal c'est plus 2-3 mois qu'il faut pour bien finaliser le projet.

[Les dix commandements version OMyBot](https://twitter.com/TeamOmybot/status/1128554678101467136) est une version plus concrète avec des conseils spécifiques pour la coupe.

N'hésitez pas à chercher/regarder ce qu'on fait les les autres équipes les années précédents, c'est motivant et ça peut donner de nouvelles idées d'actionneurs/stratégies:
* Les matchs de la Coupe de France: [Chaine Youtube Planete Science](https://www.youtube.com/@PlaneteSciencesNational/streams): 
* Les matchs de la Coupe de Belgique Robotix's: [Chaine Youtube SPARKOH!](https://www.youtube.com/@sparkoh93/streams)
* [Portail des sites web des équipes par PM-ROBOTIX](https://www.pm-robotix.eu/sites-de-la-coupe-et-des-equipes/)

## 1.3 Glossaire

**Asservissement**: Système dont le but est d'atteindre le plus rapidement possible sa valeur de consigne et de la maintenir, quelles que soient les perturbations externes.

**Billes porteuses / Billes folles**: Permettent de faire un point de contact avec le sol. Principalement utilisés aux 4 extrémitées des robots à roues différentielles.

**LiDAR**: Télémètre laser à balayage, il permet de récupérer des un nombre de points sur un plan. Il est utilisé pour la détection des mats de balise et donc pour récupérer la position des concurrents. Il peut également être utlisé au niveau du sol pour récupérer la position des éléments de jeu ou mieux détecter le gabarit des robots concurrents.

**Odométrie**: Technique d'utilisation des données de capteurs permettant d'estimer la position du robot. Utilise généralement les 2 roues codeuses pour estimer la position mais peut également être faite avec un lidar.

**Roue holonome**: Roue constituée de galets répartis sur sa périphérie, elles peuvent donc se déplacer dans toutes les directions.

**Roues codeuses / odométrie**: Roues souvent monté sur un pivot ou une glissière afin de toujours maintenir un contact avec le sol. Ces roues sont équipées d'encodeurs qui permettent une mesure de rotation pour le calcul de l'odométrie.




------------------------------------------------------------------------------------------
# 2. Ressources spécifiques à Eurobot

## 2.1 Pour bien commencer

**Tutos Robot en Carton**

> Comment on peut facilement fabriquer un robot pour participé la coupe et évidement bien s'amuser en créant des robots ^^
> 
> 1. [Comment faire un robot pour la coupe de France de Robotique](https://www.youtube.com/watch?v=VxQH7iL4wfw)
> 2. [La base roulante différentielle](https://www.youtube.com/watch?v=6YXxZ1odnGI)
> 3. [L'asservissement 1/2](https://www.youtube.com/watch?v=uEfZ-wJkynY)
> 4. [L'asservissement 2/2](https://www.youtube.com/watch?v=40OEUTVVAqI)

[Code source sut GitHub](https://github.com/RobotEnCarton/tutoRobot)

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

## 2.2 Odométrie

* [WEB - CVRA - Odometry calibration](https://cvra.ch/robot-software/howto/calibrate-odometry/)
* [VIDEO - Robotic-System - Calibrage de l'odométrie](https://www.youtube.com/watch?v=X5PMFvVecXU)
* [PDF - RCVA - Odométrie avec correction centrifuge](odometrie/RCVA-odometrie.pdf)
* [PDF - RCVA - Trajectoires courbes et odométrie, De l’importance de la différence de diamètre des deux roues
codeuses](odometrie/RCVA-Conseils_theoriques_pour_Eurobot.pdf)
* [VIDEO - RCVA - comparaison approximation linéaire/circulaire, correction centrifuge](https://www.youtube.com/watch?v=KQzfMAJyvB0)
* [VIDEO - RCVA - odométrie](https://www.youtube.com/watch?v=557l7JOs35E)

## 2.3 Roues

* [**WEB - Erich Styger - Making Perfect Sticky DIY Sumo Robot Tires**](https://mcuoneclipse.com/2017/12/28/making-perfect-sticky-diy-sumo-robot-tires/)
* [FORUM - Robotech Legends - Moulage de pneus en polyuréthane](https://www.planete-sciences.org/forums/viewtopic.php?t=18632)
* [VIDEO - Barbatronic - Moulage de pneus en silicone](https://www.youtube.com/watch?v=EGPVa1ZnXe8)
* [VIDEO - Micro Technology - test d'adhérence des roues](https://www.youtube.com/watch?v=cPfP7zyS0kU)

## 2.4 Moteurs

* [**WEB - RobotShop - Outil de Dimensionnement d'un moteur d'entrainement**](https://www.robotshop.com/community/blog/show/dimensionnement-dun-moteur-dentranement)
* [PDF - ANCR - Dimensionner ses moteurs](moteurs/ANCR-Dimensionner_ses_moteurs.pdf)
* [PDF - TechTheTroll - Dimensionnement des moteurs de propulsion](https://techthetroll.files.wordpress.com/2016/06/techthetroll-dimensionnement-des-moteurs-de-propulsion.pdf)

## 2.5 Asservissement

* [WEB - PM Robotix - Asservissement et pilotage de robot autonome]([https://www.pm-robotix.eu/2022/01/19/ameliorer-vos-regulateurs-pid/](https://www.pm-robotix.eu/2022/02/02/asservissement-et-pilotage-de-robot-autonome/))
* [WEB - PM Robotix - Améliorer vos régulateurs PID](https://www.pm-robotix.eu/2022/01/19/ameliorer-vos-regulateurs-pid/)
* [PDF - totofweb - Le PID utilisé en régulation de position et/ou de vitesse de moteurs électriques](asservissement/totofweb-PID_régulation_de_position_vitesse.pdf)
* [PDF - Microb Technology - Documentation de l'asservissement, librairie Aversive, évitement](asservissement/MicrobTechnology-wiki_asservissement_lib_aversive.pdf)
* [PDF - RCVA - Asservissement du robot à une trajectoire](http://www.rcva.fr/wp-content/uploads/2016/12/asservissement.pdf)
* [PDF - RCVA - Montée de tremplin par le robot RCVA sans terme intégral](http://www.rcva.fr/wp-content/uploads/2016/12/Montee_De_Tremplin_Sans_Terme_Integral_RCVA.pdf)
* [VIDEO - RCVA - Asservissement en rotation avec un gyromètre ADXRS453](https://www.youtube.com/watch?v=p0cm7LnMXOc)
* [VIDEO - RCVA - cours asservissement polaire](https://www.youtube.com/watch?v=JYZ_2y8k1Os)
* [PDF - TechTheTroll - Les trajectoires courbes dans la bonne humeur: de l’asservissement à la planification](https://techthetroll.files.wordpress.com/2016/07/trajectoire_courbe.pdf)
* [WEB - Implémenter un PID sans faire de calculs ! - Ferdinand Piette](http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/)

## 2.6 Robot holonome

* [WEB - Poivron Robotique - localisation (partie 1 - faisabilité)](http://poivron-robotique.fr/Robot-holonome-localisation-partie-1.html)
* [WEB - Poivron Robotique - localisation (partie 2 - les équations)](http://poivron-robotique.fr/Robot-holonome-localisation-partie-2-les-equations.html)
* [WEB - Poivron Robotique - lois de commande](http://poivron-robotique.fr/Robot-holonome-lois-de-commande.html)

## 2.7 Balises

* [PDF - totofweb - Balise infrarouge](balise/totofweb-balises_IR.pdf)
* [WEB - Barbatronic - Reflective lidar for robotic and the eurobot competition](http://fabacademy.org/2019/labs/lamachinerie/students/adrien-bracq/projects/final-project/)
* [PDF - CVRA - Development of an ultra-wide band indoor positioning system](https://github.com/cvra/robot-software/blob/1d208d0d5882d5526eef758eae61f5626291a016/uwb-beacon-firmware/doc/report.pdf)
* [PDF - CVRA - Balises laser Eurobot 2008](https://cvra.ch/ressources/misc/balise_laser.pdf)
* [PDF - Microb Technology - Faire des balises laser en buvant des bières](balise/MicrobTechnology-Faire_des_balises_laser_en_buvant_des_bieres.pdf)
* [VIDEO - ESEO - localisation par balises infrarouges](https://www.youtube.com/watch?v=bGoXEwQ0UQs)

## 2.8 Batteries

* [WEB - L’astuce batterie de PM-ROBOTIX](https://www.pm-robotix.eu/2019/07/10/lastuce-batterie-de-pm-robotix/)

## 2.9 Simulation

* [VIDEO - ESEO - simulateur de match](https://www.youtube.com/watch?v=fo-87AF2Fr4), [article sur leur site](https://robot-eseo.fr/strategie-du-robot-sur-simulateur/)

## 2.10 Planificateur de trajectoire et évitement

* [LIBRAIRIE- The Kraken Pathfinding - A tentacle-based pathfinding library for nonholonomic robotic vehicles](https://github.com/kraken-robotics/The-Kraken-Pathfinding)
* [LIBRAIRIE - PythonRobotics: Python sample codes for robotics algorithms](https://atsushisakai.github.io/PythonRobotics/)

## 2.11 Intelligence artificielle

* [VIDEO - @nesnes - Coder une IA pour Eurobot](https://www.youtube.com/channel/UCg1vR097bAzmJzeMBWl7Zzw), [depot GitHub Eurobot-AI](https://github.com/nesnes/Eurobot-AI)

## 2.12 Communication sans-fil

* [FORUM - Pourquoi éviter le WiFi 2.4GHz](https://www.planete-sciences.org/forums/viewtopic.php?f=97&t=16969)

## 2.13 Architecture des robots

* [WEB - Robotech Legends 2019](https://twitter.com/robotech34/status/1129147494859005952)
* [Librarie diagrams.net par @kmikaz51](https://drive.google.com/file/d/1W76g7xKKJeluGIWmfG8v30gIhi9cbWxQ/view?usp=sharing) pour déssiner votre propre architecture.

## 2.14 Table et les elements de jeu

* [WEB - PM Robotix - Conseils pour la pose du vinyle](https://www.pm-robotix.eu/2019/12/07/conseils-pour-la-pose-du-vinyle/)




------------------------------------------------------------------------------------------
# 3. Microcontrôleurs, ordinateur à carte unique, IDE

## 3.1 Microcontrôleurs

### 3.1.1 Arduino

[Arduino](https://www.arduino.cc/)

IDE:
* VSCode/Atom/CLion/Eclipse/SublimeText/Emacs/Vim + [extension **PlatformIO**](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)
* [Arduino IDE](https://www.arduino.cc/en/software)
* [Visual Studio Code](https://code.visualstudio.com/) + [extension Arduino](https://marketplace.visualstudio.com/items?itemName=vsciot-vscode.vscode-arduino)

### 3.1.2 STM32

* [STM32](https://www.st.com/en/evaluation-tools/stm32-mcu-mpu-eval-tools.html)
* [Cartes de développement Nucleo](https://www.st.com/en/evaluation-tools/stm32-nucleo-boards.html)

Frameworks/RTOS:
* [LL, HAL, CMSIS](https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html#products)
* [Arduino core for STM32](https://github.com/stm32duino/Arduino_Core_STM32)
* [Mbed OS](https://os.mbed.com/code/): open-source operating system for platforms using Arm microcontrollers
* [libopencm3](https://github.com/libopencm3/libopencm3): open-source firmware library for various ARM Cortex-M microcontrollers
* [ChibiOS](https://github.com/ChibiOS/ChibiOS): complete development environment for embedded applications including RTOS, an HAL, peripheral drivers, support files and tools.
* [Luos]([https://github.com/Luos-io/luos_engine](https://www.luos.io/)) Open-source and real-time orchestrator for cyber-physical-systems, to easily design, test and deploy embedded applications and digital twins. 
* [modm](https://github.com/modm-io/modm): a barebone embedded library generator, C++23 library generator for AVR and ARM Cortex-M devices 
* [Zephyr](https://github.com/zephyrproject-rtos/zephyr): Zephyr is a new generation, scalable, optimized, secure RTOS for multiple hardware architectures. 

IDE:
* VSCode/Atom/CLion/Eclipse/SublimeText/Emacs/Vim + [extension **PlatformIO**](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)
* [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
* [Keil Studio Cloud](https://www.keil.arm.com/mbed/): a modern browser-based IDE for Mbed development with compilation, code completion, linting and browser debugging. Fully compatible with Mbed OS 5 and 6 and Mbed 2. Attention cependant à avoir une solution de secours lors de la coupe au cas où il serait en maintenance quelques heures
* [Mbed Studio](https://os.mbed.com/studio/)
* [Mbed CLI 2 (mbed-tools)](https://github.com/ARMmbed/mbed-tools), [install](https://os.mbed.com/docs/mbed-os/v6.16/build-tools/install-or-upgrade.html)

### 3.1.3 Teensy

[Teensy](https://www.pjrc.com/teensy/)

IDE:
* VSCode/Atom/CLion/Eclipse/SublimeText/Emacs/Vim + [extension PlatformIO](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)
* [Arduino IDE](https://www.arduino.cc/en/software) + [extension Teensyduino](https://www.pjrc.com/teensy/teensyduino.html)

### 3.1.4 ESP32

[Cartes de développement Espressif](https://www.espressif.com/en/products/devkits), [Modules Espressif](https://www.espressif.com/en/products/modules)

Frameworks:
* [Arduino core for ESP32](https://github.com/espressif/arduino-esp32)
* [ESP-IDF](https://github.com/espressif/esp-idf): Official development framework for Espressif SoCs. 

IDE:
* VSCode/Atom/CLion/Eclipse/SublimeText/Emacs/Vim + [extension PlatformIO](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)
* [Arduino IDE](https://www.arduino.cc/en/software)

### 3.1.5 Raspberry Pi Pico

[Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/) Powerful, flexible microcontroller boards, available from $4
[RP2040](https://www.raspberrypi.com/products/rp2040/) High performance. Low cost. Small package.

* [WEB - Poivron Robotique - Les documents important](https://poivron-robotique.fr/Rpi-Pico-La-documentation.html)
* [WEB - Poivron Robotique - Installer le compilateur sous Debian 11](https://poivron-robotique.fr/Rpi-Pico-Installer-l-environnement-de-developpement-sur-Debian-11.html)
* [WEB - Poivron Robotique - Créer son projet](https://poivron-robotique.fr/Rpi-Pico-Creer-son-projet-Raspberry-Pi-Pico.html)
* [WEB - Poivron Robotique - S'installer confortablement avec VS Code](https://poivron-robotique.fr/Raspberry-Pi-Pico-S-installer-avec-VS-Code.html)

### 3.1.6 PIC/dsPIC

* [Microchip](https://www.microchip.com/)
* [Cartes de développement](https://www.microchip.com/en-us/tools-resources/evaluation-boards)
* [Microcontroleurs](https://www.microchip.com/en-us/products/microcontrollers-and-microprocessors)

librairies:
* [Code Examples](https://www.microchip.com/doclisting/CodeExamplesByFunc.aspx)

IDE:
* [MPLAB X](https://www.microchip.com/en-us/tools-resources/develop/mplab-x-ide)
* [MPLAB Compiler](https://www.microchip.com/en-us/tools-resources/develop/mplab-xc-compilers)

### 3.1.7 Autres microcontrôleurs

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
* [NanoPC & NanoPi](https://www.friendlyelec.com/index.php?route=product/category&path=69)
* [OrangePi](http://www.orangepi.org/)

[Board-DB](https://hackerboards.com/) The Single Board Computer Database: comparison website for any single-board computer (SBC), module (SoM) and Linux-supported development board. 

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
# 4. Logiciels de CAO

## 4.1 CAO électronique

* [KiCad](https://kicad-pcb.org/): Open Source Schematic Capture & PCB Design Software
* [LibrePCB](https://librepcb.org/): LibrePCB is a free & open-source software.
* [Horizon EDA](https://horizon-eda.org/): Open Source Electronic Design Automation package for printed circuit board design
* [EasyEDA](https://easyeda.com/fr): Online PCB design & circuit simulator
* [Altium Designer](https://www.altium.com/altium-designer/)
* [Eagle](https://www.autodesk.fr/products/eagle/free-download)
* [Alegro PCB Designer](https://www.cadence.com/en_US/home/tools/pcb-design-and-analysis/pcb-layout/allegro-pcb-designer.html)

Outils:
* [PCB Panelizer & Gerber tool suite](http://blog.thisisnotrocketscience.nl/projects/pcb-panelizer/)
* [PCB CheckList](https://github.com/azonenberg/pcb-checklist): Une checklist qui permet de vérifier si on à pas fait une erreur lors de la conception

## 4.2 CAO mécanique

* [Fusion 360](https://www.autodesk.com/products/fusion-360/overview)
* [SolidWorks](https://www.solidworks.com/)
* [FreeCAD](https://www.freecadweb.org/)
* [OpenSCAD](http://www.openscad.org/): The Programmers Solid 3D CAD Modeller
* [SolveSpace](http://solvespace.com/): parametric 2d/3d CAD


------------------------------------------------------------------------------------------
# 5. Actionneurs, capteurs, connectique

## 5.1 Moteurs et controleurs

### 5.1.1 Moteurs à courant continu

* [Maxon Motor](https://www.maxongroup.com/)
* [Faulhaber](https://www.faulhaber.com/)

### 5.1.2 Moteurs brushless

Controleurs de moteurs brushless:
* [SimpleFOC](https://simplefoc.com/): Arduino Compatible Open Source Field Oriented Control (FOC)
* [oDrive](https://odriverobotics.com/)
* [MJBOTS moteus](https://mjbots.com/products/moteus-r4-8)
* [Flipsky ODESC](https://flipsky.net/collections/electronic-products/products/odesc3-6-optimization-of-high-performance-brushless-motor-high-power-driver-foc-bldc-based-on-odrive)

### 5.1.3 Moteurs pas-à-pas

* [OMC-StepperOnline](https://www.omc-stepperonline.com/)
* [Trinamic](https://www.trinamic.com/)

### 5.1.4 Servomoteurs

servomoteurs classiques:
* [HITEC](https://hitecrcd.com/)

Servomoteurs intelligents:
* [HerkuleX DRS-0101](http://hovis.co.kr/guide/main_eng.html)
* [Dynamixel AX-12](http://www.robotis.us/ax-series/)

## 5.2 Roues

* [JSumo](https://www.jsumo.com/wheels)
* [Fingertech](https://www.fingertechrobotics.com/products.php?cat=Wheels+%26+Hubs)
* [BaneBots](http://www.banebots.com/category/T40P.html)

## 5.3 Encodeurs

* [encodeur rotatif à effet Hall AMS](https://ams.com/angle-position-on-axis)
* [encodeur rotatif capacitifs CUI](https://www.cuidevices.com/catalog/motion/rotary-encoders/incremental/modular)
* [encodeur rotatif optique Kubler](https://www.kuebler.com/fr/produits/mesure/codeurs/product-finder)
* [encodeur rotatif optique Broadcom](https://www.broadcom.com/products/motion-control-encoders) - par exemple, HEDR-5421-EP111 à monter à l'arrière des moteurs
* [encodeur rotatif inductif POSIC](https://www.posic.com/EN/products/rotary-encoders.html)

## 5.4 Capteurs de distance

### 5.4.1 Capteurs de distance à ultrasons

* [SICK](https://www.sick.com/fr/en/distance-sensors/ultrasonic-sensors/um18/c/g185679): UM18 utilisé à la coupe
* [Baumer](https://www.baumer.com/de/en/product-overview/distance-measurement/ultrasonic-distance-sensors/c/290): UNAM, UNDK utilisé à la coupe
* [Pepperl+Fuchs](https://www.pepperl-fuchs.com/global/en/classid_186.htm)

### 5.4.2 Télemètres laser

* [SICK Dx35](https://www.sick.com/fr/en/distance-sensors/mid-range-distance-sensors/dx35/c/g261053)
* [Baumer](https://www.baumer.com/de/en/product-overview/distance-measurement/laser-distance-sensors/c/289)

### 5.4.3 Capteurs temps de vol

[Capteurs ToF STMicroelectronics](https://www.st.com/en/imaging-and-photonics-solutions/proximity-sensors.html#products)

### 5.4.4 Capteurs photoélectriques

* [SICK](https://www.sick.com/us/en/photoelectric-sensors/c/g172752?q=:Def_Type:Product)
* [Pepperl+Fuchs](https://www.pepperl-fuchs.com/global/en/classid_8.htm)

### 5.4.5 LiDAR

* [Slamtec RPLIDAR A2](https://www.slamtec.com/en/Lidar/A2): Utilisé par de nombreuses équipes à la coupe
* [Pepperl+Fuchs](https://www.pepperl-fuchs.com/france/fr/R2000_Detection_laser_scanner.htm): R2000 utilisé à la coupe
* [Inno-maker LD06](https://www.inno-maker.com/product-category/products/ladir/)
* [Hokuyo URG-04LX-UG01](https://www.hokuyo-aut.jp/search/single.php?serial=166)
* [SICK TiM5xx](https://www.sick.com/fr/fr/solutions-de-mesure-et-de-detection/capteurs-2d-lidar/tim5xx/c/g292754)
* [Ydlidar X4](https://www.ydlidar.com/products/view/5.html)

## 5.5 Ventouses, pompes à vide, tubes

* [Coval](https://www.coval.fr/produits/)
* [Piab](https://www.piab.com/fr-FR/Produits/ventouses/)
* [Festo](https://www.festo.com/cat/fr_fr/products)
* [KNF](https://www.knf.fr/fr/produits/pompes-oem/)
* [Thomas Gardner Denver](https://www.gardnerdenver.com/en-us/thomas/gas-pumps)
* [SCHMALZ](https://www.schmalz.com/fr/technique-du-vide-pour-l-automation/composants-pour-le-vide)

## 5.6 Camera pour traitement vidéo

* [OpenMV](https://openmv.io/)
* [JeVois](https://www.jevoisinc.com/)
* [Pixy](https://pixycam.com/)
* [Intel RealSense](https://www.intel.fr/content/www/fr/fr/architecture-and-technology/realsense-overview.html)

## 5.7 Connectique et câblage

* [Wurth Electronik](https://www.we-online.com/catalog/en/em/connectors/)
* [TE](https://www.te.com/)
* [Samtec](https://www.samtec.com/)

------------------------------------------------------------------------------------------
# 6. Sites internet marchands et services

## 6.1 Mécanique

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

## 6.2 Electronique

Composants électroniques:
* [Mouser](https://www.mouser.fr/)
* [Farnell](https://fr.farnell.com/)
* [Digi-Key](https://www.digikey.fr/)
* [TME](https://www.tme.eu/fr/)
* [Arrow](https://www.arrow.com/fr-fr)
* [rutronik24](https://www.rutronik24.com/)
* [LCSC](https://www.lcsc.com/)

## 6.3 Mix électronique/mécanique

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

## 6.4 Circuit imprimé

* [Aisler](https://aisler.net/): Made in Germany
* [Eurocircuits](https://www.eurocircuits.com/) PCB prototypes & Small series, manufactured and assembled in Europe
* [OSHPark](https://oshpark.com/) Made in the USA
* [JLCPCB](https://jlcpcb.com/), [JLC PCB SMD Assembly Component Catalogue](https://yaqwsx.github.io/jlcparts/): Better parametric search for components available for JLC PCB assembly 
* [PCBWay](https://www.pcbway.com/)
* [Seeed Studio](https://www.seeedstudio.com/fusion_pcb.html)
* [PCBShopper](https://pcbshopper.com/) A price comparison site for PCB

## 6.5 Modélisme

Batteries, chargeurs, moteurs, servos, roues, ...
* [Miniplanes](https://www.miniplanes.fr/)
* [HobbyKing](https://hobbyking.com/)
* [mcmracing](https://www.mcmracing.com/)
* [EuroRC](https://www.eurorc.com/)
* [Roues Lynxmotion](https://www.robotshop.com/eu/fr/lynxmotion-roues.html)

## 6.6 Usinage

Découpe laser, impréssion 3D, fraisage, tournage, ...
* [Tolery - Fabrication en ligne de pièces métalliques sur-mesure](https://www.tolery.io/): découpe laser tôle/tube, usinage, pliage, thermolaquage, soudure
* [JohnSteel](https://www.john-steel.com/fr/)
* [Usineur.fr](https://www.usineur.fr/)
* [Protolabs](https://www.protolabs.fr/)
* [Xmake](https://www.xmake.com/)
* [Usinage boîtier](https://www.frontpanelexpress.com/)

## 6.7 Visserie

* [Bossard](https://eu.shop.bossard.com/fr/fr/)
* [Cergy-Vis](https://www.cergy-vis.fr/)
* [Vis Express](https://www.vis-express.fr/en/)
* [Bricovis](https://www.bricovis.fr/)
* [Technifix](http://www.technifix.be/)


------------------------------------------------------------------------------------------
# 7. Liens


## 7.1 Code source des équipes

* [APB Team](http://git.ni.fr.eu.org/apbteam.git/tree/)
* [ARIG](https://github.com/ARIG-Robotique)
* [CVRA](https://github.com/cvra)
* [ESEO](https://github.com/ClubRobotEseo)
* [EsialRobotik](https://github.com/EsialRobotik)
* [GRUM](https://gitlab.com/grumoncton)
* [Les Karibous](https://github.com/LesKaribous)
* [Microb Technology](https://github.com/onitake/aversive)
* [Poivron Robotique](https://git.poivron-robotique.fr/Keuronde)
* [UTCoupe](https://github.com/utcoupe)

## 7.2 Chaines youtube des équipes

* [APB Team](https://www.youtube.com/@APBTeam/videos)
* [Arig](https://www.youtube.com/@arigassociation3814/videos)
* [CVRA](https://www.youtube.com/@CVRAMedia)
* [ESEO](https://www.youtube.com/@RobotESEO/videos)
* [ESIALRobotik](https://www.youtube.com/@associationesialrobotik6587/videos)
* [Goldorak](https://www.youtube.com/@goldorak_r/videos)
* [Les Karibous](https://www.youtube.com/@equipekaribous607/videos), [Barbatronic](https://www.youtube.com/@Barbatronic/videos), [Adrien Bracq](https://www.youtube.com/@nadarbreicq/videos)
* [Microb Technology](https://www.youtube.com/@SgtKronenbourg/videos)
* [RCVA](https://www.youtube.com/@rcva/videos), [jacques coulon](https://www.youtube.com/@jacquescoulon3516/videos)
* [TURAG e.V.](https://www.youtube.com/@TURAGev/videos)
* [VRAC](https://www.youtube.com/@vrac-robotique/videos), [monowii](https://www.youtube.com/@monowii/videos)


## 7.3 Divers

[Tables de jeux, vidéos, règlements de 1994 à aujourd'hui par PM-ROBOTIX](https://www.pm-robotix.eu/accueil/les-tables-de-jeux/)

[Github cajt/list_of_robot_electronics](https://github.com/cajt/list_of_robot_electronics) A list of resources, projects and products useful for robot electronics (Motor drivers, Actuators, Battery Management, )

Connecteurs/Sertissage:
* [Common wire-to-board, wire-to-wire connectors, and crimp tools](http://www.mattmillman.com/info/crimpconnectors/)  
* [DuPont and “DuPont” connectors](http://www.mattmillman.com/info/crimpconnectors/dupont-and-dupont-connectors/)
* [Common JST Connector Types](http://www.mattmillman.com/info/crimpconnectors/common-jst-connector-types/)
* [The Electronic Connector Book](https://connectorbook.com/): A practical guide and catalog of all the connecting components used in the electronic industry.
* [Identiconn](https://connectorbook.com/identification.html) Identiconn™ Connector Identification Utility.

Bibliothèque de fichiers 3D mécanique:
* [GrabCAD](https://grabcad.com/library): Free CAD Designs, Files & 3D Models maby by the community
* [Traceparts](https://www.traceparts.com/en): Free 3D models, CAD files and 2D drawings
* [VRAC SolidWorks library](https://github.com/VRAC-team/VRAC-SolidWorks-library)

Bibliothèque d'empreintes et de symboles électronique:
* [SnapEDA](https://www.snapeda.com/) Free PCB Footprints and Schematic Symbols
* [VRAC KiCad library](https://github.com/VRAC-team/VRAC-KiCad-library)

Forums robotique:
* [Forum Robot Maker](https://www.robot-maker.com/forum/)
* [Forum Usinages.com (catégorie Robotique et Domotique)](https://www.usinages.com/forums/robotique-et-domotique.125/)

Equipes Eurobot (non listés sur le portail des équipes PM-ROBOTIX):
* [Roboterclub Aachen](https://www.roboterclub.rwth-aachen.de/)

Electronique:
* [WEB - Texas Instruments - Design tools & simulation](https://www.ti.com/design-resources/design-tools-simulation.html): conception de filtres, architecture d'arbres d'horloges, .. 
* [LS7366R 32-bit quadrature counter with serial interface](https://lsicsi.com/datasheets/LS7366R.pdf)
* [Texas Instrument LM628/LM629](http://www.ti.com/lit/ds/symlink/lm629.pdf): CI dédié l'asservissement d'un moteur, commande en position/vitesse/accélération
* [The ultimate SMD marking codes database](https://smd.yooneed.one/) Identifty SMD part by the short marking

Courroie/Guidage mécanique:
* [Générateur en ligne de roue à courroie, roues dentées, crémaillères](https://www.igus.fr/info/3d-print-gears): fichiers de CAO téléchargeables pour les imprimer en 3D ou les commander
* [Générateur de courroie GT2 imprimable en 3D](https://www.thingiverse.com/thing:3458902) avec Fusion 360
* [Système de guidage linéaire motorisé](https://gitlab.cba.mit.edu/jakeread/pgd), conçu pour être fabriqué avec de la découpe laser et de l'impression 3D

Impression 3D:
* [Print Quality Troubleshooting Guide](https://www.simplify3d.com/resources/print-quality-troubleshooting/): Warping, Dimensional Accuracy, Layer Separation and Splitting, ...

Logiciel:
* [PyRobot - light weight, high-level interface which provides hardware independent APIs for robotic manipulation and navigation by facebook research.](https://pyrobot.org/)

Littérature:
* [Elements of Robotics, Mordechai Ben-Ari, Francesco Mondada, 2018, Open Access](https://www.springer.com/gp/book/9783319625324)
* [PDF - Rich LeGrand - Closed-Loop Motion Control for Mobile Robotics](https://www.cs.hmc.edu/~dodds/projects/RobS05/XPort/XPortArticle.pdf): un Game Boy Advance, des roues holonomes, des legos et hop une base mobile
* [La RACHE, une méthodologie réaliste mais formaliste - par Sukender](https://www.la-rache.com/img/a1_rache.pdf), [Système d'unités pifométriques](https://www.la-rache.com/img/unites.pdf)
* [PDF - High-Precision Robot Odometry Using an Array of Optical Mice](https://pdfs.semanticscholar.org/37ca/1fc2dcc4c0cf860fc0b00542fc7cb59c579f.pdf) [archive](odometrie/mouse_paper_colloquium_22march11.pdf)
