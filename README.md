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

- obstacles.m
Début de classe objet pour les obstacles

Elle contient l'initialisation des drones et des obstacles, l'affichage graphique, le calcul de l'enveloppe convexe via une communication entre les drones, la création de polytope et l'intersection de ces polytopes pour générer un espace navigable pour les drones.
Sur certaines distribution, il faut lancer MATLAB avec -softwareopengl pour éviter des 'warnings' d'affichage.
Il va rester les intersections entre les obstacles et les polytopes des drones ainsi que de la gestion des formations des drones qu'ils peuvent adopter pour se déplacer.

# Bibliothèques annexes
Il est préférable d'installer les 3 bibliothèques suivantes via le panneau d'ajout d'add-ons depuis MATLAB. Ensuite, il faudra écraser les fichiers du dossier d'add-ons par celui-ci pour les librairies concernées.
- Polytopes (Bill Mckeeman) => pour la fonction plotpoly (un peu modifiée ici)
- Analyze N-dimensional Polyhedra in terms of vertices or (In)Equalities (Matt J) => pour les calculs d'intersection de polytopes
- geom3d (David Legland) => pour la création des polytopes (et des soccerball)
