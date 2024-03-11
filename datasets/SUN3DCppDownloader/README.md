## SUN3DCppDownloader

This folder contains a tool designed for downloading data from the [SUN3D server](https://sun3d.cs.princeton.edu/data/) with **multithreading parallel downloading** support. 
## Usage

Fist modify the sequences names in sequences.txt.
```
chmod +x sun3d.sh
./sun3d.sh
```
These commands will build the tool and download the sequences accordingly.

## Dependency

[**curl**](http://curl.haxx.se/). In Ubuntu: 
```
sudo apt-get install libcurl4-gnutls-dev
```
**For the WSL environment of our project, curl is already installed with ROS2.**

## Github Reference
- [PrincetonVision/SUN3DCppReader](https://github.com/PrincetonVision/SUN3DCppReader)
