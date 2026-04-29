# Configuration de bureau
Cet environnement permet de démarrer les noeuds dans un conteneur Docker sur un processeur x86.

> [!NOTE]
> Veuillez vous assurer que les fichiers `requirements_xavier.txt` et `underlay.repos` ont bien été mis à jour avant de générer l'image Docker.

Pour générer l'image, exécutez le scipt `setup.sh`. Il va s'assurer que les dépendances sont à jour avant de générer une image nommée `ros2-humble`.

Une fois l'image générée, exécutez le script `start.sh` pour démarrer le conteneur. Vous entrerez dans le terminal du conteneur et le volume du workspace sera automatiquement monté.

Pour ouvrir un autre terminal indépendant :
```bash
docker exec -it ros2-desktop /bin/bash
```