La maxi liste des ressources pour faire (un bon) robot à Eurobot
===============

Ce repo est destiné à compiler et partager les ressources (cours/vidéos/composants/discussions/liens) en lien avec la compétition [Eurobot](https://www.eurobot.org/).

Les cours, en PDF de préférence, sont sauvegardés sur le repo afin d'assurer leur longévité (même si ça reste à prouver).

Si un document vous appartient et que vous souhaitez le faire retirer, merci de nous contacter.

Vous pouvez contribuer à cette liste:

* en nous contactant sur [twitter](https://twitter.com/VRAC_robotique) ou [facebook](https://www.facebook.com/ARDICRobotique/)
* en postant un message sur le serveur discord: [Eurobot - CDR](https://discord.gg/tteC3Cp), dès qu'un lien intéressant est posté et qu'il aurait sa place ici, on s'occupe de le rajouter avec la mention (lien par @utilisateur)
* en créant une [Issue](https://github.com/VRAC-team/la-maxi-liste-ressources-eurobot/issues): on se charge de tout mettre en page au bon endroit
* en créant une [Pull Request](https://github.com/VRAC-team/la-maxi-liste-ressources-eurobot/pulls): vous vous débrouillez pour mettre la ressource au bon endroit et avec la mise en page qui va bien

**Merci aux équipes pour ces documents et aux [contributeurs de cette liste](https://github.com/VRAC-team/la-maxi-liste-ressources-eurobot/graphs/contributors) !** [Lien du git blame pour voir les modifications](https://github.com/VRAC-team/la-maxi-liste-ressources-eurobot/blame/master/README.md)


---------------
# 1. Ressources académiques spécifiques à Eurobot

## 1.1 Pour bien commencer

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
> [**EXCEL - Simulation asservissement polaire**](asservissement/Cubot-asservissement.xlsx)
>
> [EXCEL - Génération de profile trapézoidal de vitesse](asservissement/Cubot-profil_de_vitesse.xlsx)

[**Les dix commandements version OMyBot**](https://twitter.com/TeamOmybot/status/1128554678101467136)

## 1.2 Base roulante

### 1.2.1 Odométrie

[WEB - CVRA - Odometry calibration](https://cvra.ch/robot-software/howto/calibrate-odometry/)

[VIDEO - Robotic-System - Calibrage de l'odométrie](https://www.youtube.com/watch?v=X5PMFvVecXU)

[PDF - RCVA - Odométrie avec correction centrifuge](odometrie/RCVA-odometrie.pdf)

[PDF - RCVA - Trajectoires courbes et odométrie, De l’importance de la différence de diamètre des deux roues
codeuses](odometrie/RCVA-Conseils_theoriques_pour_Eurobot.pdf)

[VIDEO - RCVA - comparaison approximation linéaire/circulaire, correction centrifuge](https://www.youtube.com/watch?v=KQzfMAJyvB0)

[VIDEO - RCVA - odométrie](https://www.youtube.com/watch?v=557l7JOs35E)

[PDF - High-Precision Robot Odometry Using an Array of Optical Mice](https://pdfs.semanticscholar.org/37ca/1fc2dcc4c0cf860fc0b00542fc7cb59c579f.pdf)

### 1.2.2 Roues

[**WEB - Erich Styger - Making Perfect Sticky DIY Sumo Robot Tires**](https://mcuoneclipse.com/2017/12/28/making-perfect-sticky-diy-sumo-robot-tires/)

[FORUM - Robotech Legends - Moulage de pneus en polyuréthane](https://www.planete-sciences.org/forums/viewtopic.php?t=18632)

[VIDEO - Barbatronic - Moulage de pneus en silicone](https://www.youtube.com/watch?v=EGPVa1ZnXe8)

[VIDEO - Micro Technology - test d'adhérence des roues](https://www.youtube.com/watch?v=cPfP7zyS0kU)

### 1.2.3 Moteurs

[PDF - ANCR - Dimensionner ses moteurs](moteurs/ANCR-Dimensionner_ses_moteurs.pdf)

[PDF - TechTheTroll - Dimensionnement des moteurs de propulsion](https://techthetroll.files.wordpress.com/2016/06/techthetroll-dimensionnement-des-moteurs-de-propulsion.pdf)

[WEB - RobotShop - Outil de Dimensionnement d'un moteur d'entrainement](https://www.robotshop.com/community/blog/show/dimensionnement-dun-moteur-dentranement) (lien par @kmikazevolutek)

[WEB - Faulhaber - Drive Selection, outil d'aide au dimensionnement](https://www.faulhaber.com/fr/driveselection/fdst/)

## 1.3 Asservissement

[PDF - totofweb - Le PID utilisé en régulation de position et/ou de vitesse de moteurs électriques](asservissement/totofweb-PID_régulation_de_position_vitesse.pdf)

[PDF - Microb Technology - Documentation de l'asservissement, librairie Aversive, évitement](asservissement/MicrobTechnology-wiki_asservissement_lib_aversive.pdf)

[PDF - RCVA - Asservissement du robot à une trajectoire](http://www.rcva.fr/wp-content/uploads/2016/12/asservissement.pdf)

[PDF - RCVA - Montée de tremplin par le robot RCVA sans terme intégral](http://www.rcva.fr/wp-content/uploads/2016/12/Montee_De_Tremplin_Sans_Terme_Integral_RCVA.pdf)

[VIDEO - RCVA - Asservissement en rotation avec un gyromètre ADXRS453](https://www.youtube.com/watch?v=p0cm7LnMXOc)

[VIDEO - RCVA - cours asservissement polaire](https://www.youtube.com/watch?v=JYZ_2y8k1Os)

[PDF - TechTheTroll - Les trajectoires courbes dans la bonne humeur: de l’asservissement à la planification](https://techthetroll.files.wordpress.com/2016/07/trajectoire_courbe.pdf)

[PDF - Rich LeGrand - Closed-Loop Motion Control for Mobile Robotics](https://www.cs.hmc.edu/~dodds/projects/RobS05/XPort/XPortArticle.pdf): un Game Boy Advance, des roues holonomes, des legos

[WEB - Implémenter un PID sans faire de calculs ! - Ferdinand Piette](http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/)

## 1.4 Balises

[PDF - totofweb - Balise infrarouge](balise/totofweb-balises_IR.pdf)

[WEB - Barbatronic - Reflective lidar for robotic and the eurobot competition](http://fabacademy.org/2019/labs/lamachinerie/students/adrien-bracq/projects/final-project/)

[PDF - CVRA - Development of an ultra-wide band indoor positioning system](https://github.com/cvra/robot-software/blob/1d208d0d5882d5526eef758eae61f5626291a016/uwb-beacon-firmware/doc/report.pdf)

[PDF - CVRA - Balises laser Eurobot 2008](https://cvra.ch/ressources/misc/balise_laser.pdf)

[PDF - Microb Technology - Faire des balises laser en buvant des bières](balise/MicrobTechnology-Faire_des_balises_laser_en_buvant_des_bieres.pdf)

[VIDEO - ESEO - localisation par balises infrarouges](https://www.youtube.com/watch?v=bGoXEwQ0UQs)

## 1.5 Simulation

[VIDEO - ESEO - simulateur de match](https://www.youtube.com/watch?v=fo-87AF2Fr4), [article sur leur site](https://robot-eseo.fr/strategie-du-robot-sur-simulateur/)

## 1.6 Planificateur de trajectoire et évitement

[LIBRAIRIE- The Kraken Pathfinding - A tentacle-based pathfinding library for nonholonomic robotic vehicles](https://github.com/kraken-robotics/The-Kraken-Pathfinding)

[LIBRAIRIE - PythonRobotics: Python sample codes for robotics algorithms](https://atsushisakai.github.io/PythonRobotics/)

## 1.7 Intelligence Artificielle

[VIDEO - alexnesnes - Coder une IA pour Eurobot](https://www.youtube.com/channel/UCg1vR097bAzmJzeMBWl7Zzw), [depot GitHub Eurobot-AI](https://github.com/nesnes/Eurobot-AI)

## 1.8 Communication sans-fil

[FORUM - Pourquoi éviter le WiFi 2.4GHz](https://www.planete-sciences.org/forums/viewtopic.php?f=97&t=16969)

## 1.9 Architecture des robots

[WEB - Robotech Legends 2019](https://twitter.com/robotech34/status/1129147494859005952)

[Librarie pour Diagrams.net](https://drive.google.com/file/d/1W76g7xKKJeluGIWmfG8v30gIhi9cbWxQ/view?usp=sharing) (anciennement draw.io) pour déssiner votre propre architecture. (lien par @kmikazevolutek)

---------------
# 2. Composants, fabricants, sites marchands

## 2.1 Cartes de développement et IDE

### 2.1.1 Microcontrôleurs:

[Arduino](https://www.arduino.cc/)

> IDE:
> * [Visual Studio Code](https://code.visualstudio.com/): rapide, modulaire, intégration Arduino avec une extension [tutoriel vidéo par Eric Peronnin](https://www.youtube.com/watch?v=o2aD2kwinJM&list=PLuQznwVAhY2XaAnqx5CExigcnlw-sK4nP)
> * [Atom](https://atom.io/): Léger, Open-source, compatible avec la plupart des uC grace à [PlatformIO](https://platformio.org/)

[STM32](https://www.st.com/en/evaluation-tools/stm32-mcu-mpu-eval-tools.html)

> outils:
> * [LL, HAL, CMSIS](https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html#products)
> * [Mbed OS](https://os.mbed.com/code/): open-source operating system for platforms using Arm microcontrollers
> * [libopencm3](https://github.com/libopencm3/libopencm3): open-source firmware library for various ARM Cortex-M microcontrollers
> * [ChibiOS](https://github.com/ChibiOS/ChibiOS): complete development environment for embedded applications including RTOS, an HAL, peripheral drivers, support files and tools.

> IDE:
> * [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html): Combinaison de l'IDE TrueSTUDIO avec STM32CubeMX
> * [System Workbench for STM32](https://www.st.com/en/development-tools/sw4stm32.html): Supporte mbed (nécéssite une [manip pour corriger quelques erreurs](ide/fix_mbed_import_sw4stm32.txt))
> * [Mbed online compiler](https://os.mbed.com/): IDE en ligne, pas de débogueur, intègre un gestionnaire de version. Attention cependant à avoir une solution de secour lors de la coupe au cas où il serait en maintenance quelques heures
> * [Mbed Studio](https://os.mbed.com/studio/)


[Teensy](https://www.pjrc.com/teensy/)

[WEB - The amazing $1 microcontroller by Jay Carlson](https://jaycarlson.net/microcontrollers/)

### 2.1.2 FPGA

[WEB - Cheap FPGA Development Boards by Joel W.](https://joelw.id.au/FPGA/CheapFPGADevelopmentBoards). (lien par @alf_arobase)

### 2.1.3 Autres cartes de dev

Ordinateur à carte unique:

* [RaspberryPi](https://www.raspberrypi.org/products/)
* [BeagleBone](https://beagleboard.org/)
* [NVidia Jetson](https://www.nvidia.com/fr-fr/autonomous-machines/embedded-systems/jetson-nano/)

[Cypress PSoC 6](https://www.cypress.com/documentation/development-kitsboards/psoc-6-ble-prototyping-kit-cy8cproto-063-ble), [Cypress PSoC 5](https://www.cypress.com/documentation/development-kitsboards/cy8ckit-059-psoc-5lp-prototyping-kit-onboard-programmer-and)

### 2.1.4 Outils pour le dev

[PlatformIO](https://platformio.org/): Extension pour de [nombreux IDE](https://platformio.org/install/integration) (VSCode, Atom, CLion, CodeBlocks, ...) qui supporte plus de 800 cartes (dont STM32, Teensy, Arduino, ESP32, PIC32, ...)

[Serial Port Plotter](https://github.com/CieNTi/serial_port_plotter): windows application that displays real time data from serial port, built with Qt

[ScriptCommunicator](https://github.com/szieke/ScriptCommunicator_serial-terminal): scriptable cross-platform data terminal which supports serial port (RS232, USB to serial), UDP, TCP client/server, SPI, I2C and CAN

[YAT: Yet Another Terminal](https://sourceforge.net/projects/y-a-terminal/): windows application supports RS-232/422/423/485 as well as TCP/IP Client/Server/AutoSocket, UDP/IP Client/Server/PairSocket and USB Ser/HID

[PlotJuggler](https://github.com/facontidavide/PlotJuggler): QT5 based application to display time series in plots, ROS integration

[MobaXterm](https://mobaxterm.mobatek.net/): Enhanced terminal for Windows with X11 server, tabbed SSH client, network tools and much more

## 2.2 Moteurs et controleurs de moteurs

### 2.2.1 Moteurs DC & Brushless

* [Controleur brushless oDrive](https://odriverobotics.com/)
* [LM628/LM629 Precision Motion Controller](http://www.ti.com/lit/ds/symlink/lm629.pdf)
* [Faulhaber](https://www.faulhaber.com/)
* [Maxon Motor](https://www.maxongroup.com/)

### 2.2.2 Moteurs pas-à-pas

* [OMC-StepperOnline](https://www.omc-stepperonline.com/)
* [Trinamic](https://www.trinamic.com/)

### 2.2.3 Servomoteurs

* [HITEC](https://hitecrcd.com/)
* [HerkuleX DRS-0101](http://hovis.co.kr/guide/main_eng.html)
* [Dynamixel AX-12](http://www.robotis.us/ax-series/)

## 2.3 Roues

* [JSumo](https://www.jsumo.com/wheels)
* [Fingertech](https://www.fingertechrobotics.com/products.php?cat=Wheels+%26+Hubs)
* [BaneBots](http://www.banebots.com/category/T40P.html)
* [Lynxmotion](https://www.robotshop.com/eu/fr/lynxmotion-roues.html)
* [mcmracing](https://www.mcmracing.com/fr/47-pneus-inserts): Pneus et inserts de modélisme (lien par Rodger de Labo404)
* [EuroRC](https://www.eurorc.com/category/84/wheels): Livraison très rapide

## 2.4 Encodeurs

* [rotatif à effet Hall AMS](https://ams.com/angle-position-on-axis)
* [rotatif capacitifs CUI](https://www.cuidevices.com/catalog/motion/rotary-encoders/incremental/modular)
* [rotatif optique Kubler](https://www.kuebler.com/fr/produits/mesure/codeurs/product-finder)
* [rotatif inductif POSIC](https://www.posic.com/EN/products/rotary-encoders.html)
* [LS7366R 32-bit quadrature counter with serial interface](https://lsicsi.com/datasheets/LS7366R.pdf)

## 2.5 Capteurs de distance, lidars

* [Capteurs de distance à moyenne portée SICK, laser classe 1 ou 2, ou infra-rouge](https://www.sick.com/fr/fr/capteurs-de-distance/capteurs-de-distance-a-moyenne-portee/c/g176373)
* [Capteurs Time-of-Flight STMicroelectronics](https://www.st.com/en/imaging-and-photonics-solutions/proximity-sensors.html#products)
* [Slamtec RPLida A1](https://www.slamtec.com/en/Lidar/A1)
* [Ydlidar X4](https://www.ydlidar.com/products/view/5.html)

## 2.6 Ventouses, pompes à vide, composants pour le vide

* [Coval](https://www.coval.fr/produits/)
* [Piab](https://www.piab.com/fr-FR/Produits/ventouses/)
* [Festo](https://www.festo.com/cat/fr_fr/products)
* [KNF](https://www.knf.fr/fr/produits/pompes-oem/)
* [Thomas Gardner Denver](https://www.gardnerdenver.com/en-us/thomas/gas-pumps)
* [SCHMALZ](https://www.schmalz.com/fr/technique-du-vide-pour-l-automation/composants-pour-le-vide)

## 2.7 Camera pour traitement vidéo

* [OpenMV](https://openmv.io/)
* [JeVois](https://www.jevoisinc.com/)
* [Pixy](https://pixycam.com/)
* [Intel RealSense](https://www.intel.fr/content/www/fr/fr/architecture-and-technology/realsense-overview.html)

## 2.7 Connectique et câblage

* [Wurth Electronik](https://www.we-online.com/catalog/en/em/connectors/)
* [TE](https://www.te.com/)
* [Samtec](https://www.samtec.com/)

## 2.8 Sites marchands en vrac

Modélisme, pour les batteries, chargeurs, moteurs DC, moteurs brushless, contrôleurs, servos, ...

* [Miniplanes](https://www.miniplanes.fr/)
* [HobbyKing](https://hobbyking.com/)
* [mcmracing](https://www.mcmracing.com/)
* [EuroRC](https://www.eurorc.com/)

Uniquement mécanique:

* [Misumi](https://fr.misumi-ec.com/)
* [Motedis](https://www.motedis.fr/): profilées aluminium
* [MakerBeam](https://www.makerbeam.com/): profilées aluminium
* [Systeal](https://www.systeal.com/fr/): profilées aluminium
* [Technic-Achat](https://www.technic-achat.com/): profilées aluminium
* [123-Roulements](https://www.123roulement.be/): roulements

Visserie:

* [Bossard](https://eu.shop.bossard.com/fr/fr/)
* [Cergy-Vis](https://www.cergy-vis.fr/)
* [Vis Express](https://www.vis-express.fr/en/)
* [Bricovis](https://www.bricovis.fr/)
* [Technifix](http://www.technifix.be/)

Mixes mécanique/électronique:

* [RS Components](https://fr.rs-online.com/)
* [Pololu](https://www.pololu.com/)
* [RobotShop](https://www.robotshop.com/)
* [Conrad](https://www.conrad.fr/)
* [GoTronic](https://www.gotronic.fr/)
* [Lextronic](https://www.lextronic.fr/)
* [Distrelec](https://www.distrelec.be/)

Plutôt électronique:

* [Adafruit](https://www.adafruit.com/)
* [Sparkfun](https://www.sparkfun.com/)
* [Antratek](https://www.antratek.com/)
* [Watterott](https://shop.watterott.com/)

Uniquement électronique:

* [Mouser](https://www.mouser.fr/)
* [Farnell](https://fr.farnell.com/)
* [Digi-Key](https://www.digikey.fr/)
* [TME](https://www.tme.eu/fr/)
* [Arrow](https://www.arrow.com/fr-fr)
* [rutronik24](https://www.rutronik24.com/)

Fabricants de circuits imprimés:

* [Aisler](https://aisler.net/)
* [Eurocircuits](https://www.eurocircuits.com/)
* [OSHPark](https://oshpark.com/)
* [PCBWay](https://www.pcbway.com/): Assemblage possible
* [Seeed Studio](https://www.seeedstudio.com/fusion_pcb.html): Assemblage possible
* [JLCPCB](https://jlcpcb.com/): Assemblage possible (attention, limité à leur catalogue de pièces) => compter au moins 1 mois de livraison pour la livraison standard
* Comparateur de prix: [PCBShopper](https://pcbshopper.com/)

Service d'usinage:

* [JohnSteel](https://www.john-steel.com/fr/)
* [Usineur.fr](https://www.usineur.fr/)
* [Protolabs](https://www.protolabs.fr/)
* [Xmake](https://www.xmake.com/)
* [Usinage boîtier](https://www.frontpanelexpress.com/)

Impression Vinyl:

* [Pixart printing](https://fr.pixartprinting.be/)

Plastiques (plexi, acrylique, dibond, etc...):

* [Lacrylicshop](https://www.decoupe-plexi-sur-mesure.com/)
* [Plexiglas-shop](https://www.plexiglas-shop.com/)
* [SuperPlastic](https://www.superplastic.be/)
* [Pyrasied](https://www.pyrasied.nl/)

Filaments 3D spéciaux:

* [Volcano PLA](https://www.formfutura.com/shop/product/volcano-pla-black-980): PLA à haute resistance à "re-cuire" après impression pour solidifier la liaison inter-couches


---------------
# 3. Logiciels

## PCB

* [KiCad](https://kicad-pcb.org/): A Cross Platform and Open Source Electronics Design Automation Suite, [tutoriel vidéo par Eric Peronnin](https://www.youtube.com/watch?v=C9EWrKw9Qz8&list=PLuQznwVAhY2VoayfSraJjI-Yr2OSGmFKt)
* [LibrePCB](https://librepcb.org/): A new, powerful and intuitive EDA tool for everyone
* [EasyEDA](https://easyeda.com/fr): Online PCB Design Tool
* [Eagle](https://www.autodesk.fr/products/eagle/free-download)
* [Altium Designer](https://www.altium.com/altium-designer/)
* [Alegro PCB Designer](https://www.cadence.com/en_US/home/tools/pcb-design-and-analysis/pcb-layout/allegro-pcb-designer.html)

outils:

* [PCB panelizer & Gerber tool suite](http://blog.thisisnotrocketscience.nl/projects/pcb-panelizer/), [tutoriel vidéo par Christian Hortolland](https://www.youtube.com/watch?v=c5XWobI4Eog). (lien par @King0vCh0uffe)
* [gerbv](http://gerbv.geda-project.org/): A Free/Open Source Gerber Viewer
* [PCB CheckList](https://github.com/azonenberg/pcb-checklist): A complete PCB checklist

## CAO

* [Fusion 360](https://www.autodesk.com/products/fusion-360/overview)
* [SOLIDWORKS](https://www.solidworks.com/)
* [FreeCAD](https://www.freecadweb.org/)
* [OpenSCAD](http://www.openscad.org/): The Programmers Solid 3D CAD Modeller
* [SOLVESPACE](http://solvespace.com/): parametric 2d/3d CAD


---------------
# 4. Autres liens

## 4.1 Codes sources des équipes

* [CVRA](https://github.com/cvra)
* [Microb Technology](https://github.com/onitake/aversive)
* [ESEO](https://github.com/ClubRobotEseo)
* [EsialRobotik](https://github.com/EsialRobotik)
* [ARIG Robotique](https://github.com/ARIG-Robotique)
* [APB Team](http://git.ni.fr.eu.org/apbteam.git/tree/)
* [GRUM](https://gitlab.com/grumoncton)

## 4.2 Liens en vrac

### 4.2.1 Forums

* [Forum Planete Science / Coupe de France de robotique](https://www.planete-sciences.org/forums/)
* [Forum Robot Maker](https://www.robot-maker.com/forum/)
* [Forum Usinages.com (catégorie Robotique et Domotique)](https://www.usinages.com/forums/robotique-et-domotique.125/)

### 4.2.2 Électronique

* [VIDEO - 10 erreurs de soudure à éviter](https://www.youtube.com/watch?v=Fp37DPZVdRI) (lien par @alf_arobase)
* [VIDEO - Eric Peronnin (prof d'IUT) - Cours d'électronique (conception, CAO, FPGA)](https://www.youtube.com/channel/UCe3v5cVACw-5BKQOcwUaM8w/playlists)
* [WEB - SONELEC-MUSIQUE - Réalisations, Théorie et bases](https://www.sonelec-musique.com/electronique.html)
* [WEB - Texas Instruments - Design tools & simulation](https://www.ti.com/design-resources/design-tools-simulation.html)

### 4.2.3 Transmission de puissance et guidage

[Génération de pignon et crémaillère](https://www.igus.fr/info/3d-print-gears), fichiers de CAO téléchargeables pour les imprimer en 3D ou les ajouter dans un assemblage

[Customizable Timing Belt in Fusion 360 by COM3](https://layershift.xyz/customgt2belt/), [GT2 printable Belt generator sur Thingiverse](https://www.thingiverse.com/thing:3458902). (lien par @Barbatronic)

[Calculateur de longueur de courroie](https://www.bbman.com/belt-length-calculator/)

[Système de guidage linéaire motorisé](https://gitlab.cba.mit.edu/jakeread/pgd), conçu pour être fabriqué avec de la découpe laser et de l'impression 3D (lien par @Barbatronic)

### 4.2.4 Autres

[Portail des sites web des équipes par PM-ROBOTIX](https://www.pm-robotix.eu/sites-de-la-coupe-et-des-equipes/)

[Elements of Robotics, Mordechai Ben-Ari, Francesco Mondada, 2018, Open Access](https://www.springer.com/gp/book/9783319625324)

[PyRobot - light weight, high-level interface which provides hardware independent APIs for robotic manipulation and navigation by facebook research.](https://pyrobot.org/)

[VIDEO - Robert Cowan - Montage de réducteur planétaire sur un moteur brushless](https://www.youtube.com/watch?v=TfYZbjtgO0k)

[WEB - lecture "Applied Artificial Intelligence" at the University of Applied Sciences Esslingen](https://github.com/MrDio/Applied-AI-Technologies)

[La RACHE, une méthodologie réaliste mais formaliste - par Sukender](https://www.la-rache.com/img/a1_rache.pdf), [Système d'unités pifométriques](https://www.la-rache.com/img/unites.pdf)
