# Ressources pour faire un bon robot à Eurobot

Ce repo est destiné à compiler et partager les ressources (cours/vidéos/composants/discussions/liens) en lien avec la compétition [Eurobot](https://www.eurobot.org/).

Les cours, en PDF de préférence, sont sauvegardés sur le repo afin d'assure leur longévité (même si ça reste à prouver).

Si un document vous appartient et que vous souhaitez le faire retirer, merci de nous contact.

Vous pouvez contribuer en créant une [Pull Request](https://github.com/VRAC-team/ressources/pulls)

## Pour bien commencer

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
> [**PDF - RCVA - Réflexions sur un robot Eurobot**](dossier/RCVA-Réflexions_sur_un_robot_Eurobot.pdf)

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

## Base roulante

### Odométrie

[WEB - CVRA - Odometry calibration](https://cvra.ch/robot-software/howto/calibrate-odometry/)

[PDF - RCVA - Odométrie avec correction centrifuge](odometrie/RCVA-odometrie.pdf)

[PDF - RCVA - Trajectoires courbes et odométrie, De l’importance de la différence de diamètre des deux roues
codeuses](odometrie/RCVA-Conseils_theoriques_pour_Eurobot.pdf)

[VIDEO - RCVA - comparaison approximation linéaire/circulaire, correction centrifuge](https://www.youtube.com/watch?v=KQzfMAJyvB0)

[VIDEO - RCVA - odométrie](https://www.youtube.com/watch?v=557l7JOs35E)

[WEB PDF - Enhanced Positioning SystemsUsing Optical Mouse Sensors](https://www.researchgate.net/publication/272238238_Enhanced_Positioning_Systems_Using_Optical_Mouse_Sensors)

[WEB PDF - High-Precision Robot Odometry Using an Array of Optical Mice](https://pdfs.semanticscholar.org/37ca/1fc2dcc4c0cf860fc0b00542fc7cb59c579f.pdf)

#### Encodeurs

* [COMPOSANT - Encodeur rotatif à effet Hall AMS](https://ams.com/angle-position-on-axis)
* [COMPOSANT - Encodeur rotatif capacitifs CUI](https://www.cuidevices.com/catalog/motion/rotary-encoders/incremental/modular)
* [COMPOSANT - Encodeur rotatif optique Kubler](https://www.kuebler.com/fr/produits/mesure/codeurs/product-finder)
* [COMPOSANT - Encodeur rotatif inductif POSIC](https://www.posic.com/EN/products/rotary-encoders.html)
* [COMPOSANT - LS7366R 32-bit quadrature counter with serial interface] (https://lsicsi.com/datasheets/LS7366R.pdf)

### Roues

[FORUM - Robotech Legends - Moulage de pneus en polyuréthane](https://www.planete-sciences.org/forums/viewtopic.php?t=18632)

[VIDEO - Barbatronic - Moulage de pneus en silicone](https://www.youtube.com/watch?v=EGPVa1ZnXe8)

[VIDEO - Micro Technology - test d'adhérence des roues](https://www.youtube.com/watch?v=cPfP7zyS0kU)

* [COMPOSANT - Roues JSumo](https://www.jsumo.com/wheels)
* [COMPOSANT - Roues BaneBots](http://www.banebots.com/category/T40P.html)
* [COMPOSANT - Roues Lynxmotion](https://www.robotshop.com/eu/fr/lynxmotion-roues.html)

### Moteurs

[PDF - ANCR - Dimensionner ses moteurs](moteurs/ANCR-Dimensionner_ses_moteurs.pdf)

[PDF - TechTheTroll - Dimensionnement des moteurs de propulsion](moteurs/TechTheTroll-Dimensionnement_des_moteurs_de_propulsion.pdf)

[VIDEO - Robert Cowan - Montage de réducteur planétaire sur un moteur brushless](https://www.youtube.com/watch?v=TfYZbjtgO0k)

* [COMPOSANT - Controleur brushless oDrive](https://odriverobotics.com/)
* [COMPOSANT - Controleurs Pololu](https://www.pololu.com/category/9/motion-control-modules)
* [COMPOSANT - Moteurs brushless HobbyKing](https://hobbyking.com/fr_fr/power-systems/electric-motors/brushless-motors.html)

## Asservissement

[PDF - TechTheTroll - Les trajectoires courbes dans la bonne humeur: de l’asservissement à la planification](asservissement/TechTheTroll-trajectoire_courbe.pdf)

[PDF - totofweb - Le PID utilisé en régulation de position et/ou de vitesse de moteurs électriques](asservissement/totofweb-PID_régulation_de_position_vitesse.pdf)

[PDF - Microb Technology - Documentation de l'asservissement, librairie Aversive, évitement](asservissement/MicrobTechnology-wiki_asservissement_lib_aversive.pdf)

[PDF - RCVA - Asservissement du robot à une trajectoire](asservissement/RCVA-asservissement.pdf)

[PDF - RCVA - Montée de tremplin par le robot RCVA sans terme intégral](asservissement/RCVA-Montee_de_tremplin_sans_terme_integral.pdf)

[VIDEO - RCVA - cours asservissement polaire](https://www.youtube.com/watch?v=JYZ_2y8k1Os)

[COMPOSANT - LM628/LM629 Precision Motion Controller](http://www.ti.com/lit/ds/symlink/lm629.pdf)

### Planificateur de trajectoire

[LIBRARIE - The Kraken Pathfinding - A tentacle-based pathfinding library for nonholonomic robotic vehicles](https://github.com/kraken-robotics/The-Kraken-Pathfinding)

## Balises

[PDF - totofweb - Balise infrarouge](balise/totofweb-balises_IR.pdf)

[WEB - Barbatronic - Reflective lidar for robotic and the eurobot competition](http://fabacademy.org/2019/labs/lamachinerie/students/adrien-bracq/projects/final-project/)

[PDF - CVRA - Development of an ultra-wide band indoor positioning system](https://github.com/cvra/robot-software/blob/1d208d0d5882d5526eef758eae61f5626291a016/uwb-beacon-firmware/doc/report.pdf)

[PDF - Microb Technology - Faire des balises laser en buvant des bières](balise/MicrobTechnology-Faire_des_balises_laser_en_buvant_des_bieres.pdf)

[VIDEO - ESEO - localisation par balises infrarouges](https://www.youtube.com/watch?v=bGoXEwQ0UQs)

## Simulation

[VIDEO - ESEO - simulateur de match](https://www.youtube.com/watch?v=fo-87AF2Fr4)

## Communication sans-fil

[FORUM - Pourquoi éviter le WiFi 2.4GHz](https://www.planete-sciences.org/forums/viewtopic.php?f=97&t=16969)



## Codes sources des équipes

[CVRA](https://github.com/cvra)

[Microb Technology](https://github.com/onitake/aversive)

[ESEO](https://github.com/ClubRobotEseo)

[EsialRobotik](https://github.com/EsialRobotik)

[ARIG Robotique](https://github.com/ARIG-Robotique)

## Autres liens

[Forum Planete Science / Coupe de France de robotique](https://www.planete-sciences.org/forums/)

[Portail des sites web des équipes par PM-ROBOTIX](https://www.pm-robotix.eu/sites-de-la-coupe-et-des-equipes/)

[Elements of Robotics, Mordechai Ben-Ari, Francesco Mondada, 2018, Open Access](https://www.springer.com/gp/book/9783319625324)