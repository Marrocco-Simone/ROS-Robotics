# ROS-Robotics
This repo is not finished yet, nor at an acceptable status. We are working on a project for Introduction to Robotics course at University of Trento.

# Start Git
Create una nuova cartella dove avere dentro due cartelle, una con il vostro cognome e una chiamata **localcopy**  
Lo schema delle cartelle deve essere così:
```
../principale  
    |-/<cognome>  
    |-/localcopy
```
Usate questi comandi dalla cartella principale per iniziare il git
```
git init
git commit -m "first commit"
git branch -M main
git remote add origin git@github.com:Marrocco-Simone/ROS-Robotics.git
git pull
git push -u origin main
```
Se non dovesse funzionare remote, controllate di aver collegato il vostro terminale a github da un tutorial qualsiasi
Mettete tutti i vostri file, compresa la cartella catkin_ws, dentro **localcopy** e lavorate li'
Quando siete pronti a caricare il vostro lavoro, copiate da **localcopy** alla cartella col vostro lavoro e usate questi comandi
```
git pull
git add .
git commit -m "scrivete quello che avete fatto"
git push
```
