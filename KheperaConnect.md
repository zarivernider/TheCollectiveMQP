# Connecting to the Khepera

## Run the Docker container
Go to your local directory that has the Dockerfile in it <br />
*Note: if you get a permissions error use sudo* <br />
`$ docker build -t nestlab/kheperaiv .` <br />
`$ docker run -it -v $(pwd):/work nestlab/kheperaiv bash -l` <br />
boom! you're in

## Join the same WiFi network as the Khepera
WiFi Network: NESTLab <br />
Password: nkvvw35634 <br /><br />
*If the WiFi network is not on:* 
- *Turn on the computer across from the lab*
- *Plug in the power supply next to the computer*
- *Flip the power supply switch to ON*
- *Plug in the router*

## Connect to the Khepera
*We have been assigned the Khepera05, but these instructions are valid for any Khepera*

### Install vim:
*Dockerfile in Github automatically does this* <br /><br />
`$ sudo apt-get install vim`

### Make a config file:
*Only need to do this the first time you setup the connection, after that the Dockerfile takes care of this* <br /><br />
`$ mkdir ~/.ssh` <br />
`$ vim ~/.ssh/config` 

In the config file paste the following: <br />
``` 
Host khepera01
HostName 192.168.1.201
User root
Host khepera02
HostName 192.168.1.202
User root
Host khepera03
HostName 192.168.1.203
User root
Host khepera04
HostName 192.168.1.204
User root
Host khepera05
HostName 192.168.1.205
User root
Host khepera06
HostName 192.168.1.206
User root
Host khepera07
HostName 192.168.1.207
User root
Host khepera08
HostName 192.168.1.208
User root
Host khepera09
HostName 192.168.1.209
User root
Host khepera10
HostName 192.168.1.210
User root 
```
### SSH into a robot:
`$ ssh khepera05`
