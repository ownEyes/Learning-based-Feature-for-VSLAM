## SUN3DCppDownloader

This folder is a tool to download data from the SUN3D server. (https://sun3d.csail.mit.edu/data/). 
## Usage

Fist modify the sequences names in sequences.txt.
```
chmod +x sun3d.sh
./sun3d.sh
```
These commands will build the tool and download the sequences accordingly.

## Dependency

curl (http://curl.haxx.se/). In Ubuntu: sudo apt-get install libcurl4-gnutls-dev

**For the WSL environment of our project, curl is already installed with ROS2.**
