# undergraduate-ros-project 

  
## Managing binary package dependencies 
A binary package can be installed without the need to build it. Official ROS packages can be installed as binary with the following: 
``` 
sudo apt install ros-{ROS_DISRO}-{PACKAGE_NAME} 
``` 
For example,   
``` 
sudo apt install ros-humble-control 
``` 
However, it is best practice to add all binary packages to the package xml (see [example](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#customize-package-xml)) and then install binary packages with  
``` 
rosdep install --from-paths src --ignore-src -y 
``` 
Note, the above command must be run at the root of the ROS workspace. 

  
## Managing source package dependencies 
A source package must be build in order to use it. It is recommended to manage these packages using a .repos file. In this file, git repositories are listed in a .yaml file format and can be downloaded using vcs. To install vcs you can run: 
``` 
sudo apt-get install python3-vcstool 
``` 
Once installed, you can run the following to download all of the dependencies. 
``` 
vcs import < external.repos.yaml 
``` 
## Making Docker Image and Running in a Container
Building the docker image from dir docker and naming it nao with tag latest
```
sudo docker build docker -it nao:latest
```

Uses docker image from above to make a container
```
cd docker
```
```
sudo docker compose up
```

 In a new terminal (or set docker container to run in the background).
```
sudo docker exec -it docker-nao_controller-1 bash
```
You'll still need to source ROS, but all external packages will already be downloaded.
Make sure to source again after building in the newly built install folder

