# Dockerfile to change time on device from gps topics
In that case it is also needed to have modified ublox package and script.sh file to run roslaunch.
To run that image use command:

```bash 
docker run -it --privileged  --device=/dev/gps -v /etc/timezone:/etc/timezone:ro -v /etc/localtime:/etc/localtime:ro --ipc=host  image_name
```