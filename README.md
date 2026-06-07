# An Inverted Pendulum Cart Robot
This project holds the source code for a inverted pendulum car robot. The code is written primarily in cpp and is intended to be used with the mjbots family of tools. 

## Installation 
The project uses **gcc** and **cmake**, they must be installed in order to build.

Build steps are as follows
```bash
cd build
cmake .. # init 

cmake --build .
./bot # might need sudo
```

**Remote Development**: use x11 forwarding on pi and other SBC. Here are common short cuts.
```bash
sudo nmap -sn 192.168.1.0/24 # find pi on network
ssh -Y pi@address # allows trusted x11 forwarding
```