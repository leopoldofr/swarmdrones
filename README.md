# swarmdrones
Implémentation pour un algorithme de controle de drones (MIT) sous MATLAB
(https://web.stanford.edu/~schwager/MyPapers/Alonso-MoraEtAlICRA16FormationNavigation.pdf)

- drone.m
La classe objet pour créer et manipuler un drone

- point.m
La classe object pour créer et manipuler un point (x,y,z)

- convexHull2D.m
La classe objet pour créer et manipuler et calculer les enveloppes convexes (convex hull) de chaque drone.

- main.m
La classe principale pour lancer la simulation.

Elle contient l'initialisation des drones et des obstacles, l'affichage graphique, le calcul de l'enveloppe convexe via une communication entre les drones, la création de polytope et l'intersection de ces polytopes pour générer un espace navigable pour les drones.
Sur certaines distribution, il faut lancer MATLAB avec -softwareopengl pour éviter des 'warnings' d'affichage.
Il va rester les intersections entre les obstacles et les polytopes des drones ainsi que de la gestion des formations des drones qu'ils peuvent adopter pour se déplacer.
