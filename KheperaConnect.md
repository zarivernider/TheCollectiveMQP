Go to your local directory that has the Dockerfile in it
Note: if you get a permissions error use sudo
$ docker build -t nestlab/kheperaiv . 
$ docker run -it -v $(pwd):/work nestlab/kheperaiv bash -l

boom! you're in
And then to connect to khepera, join the NESTLab Wi-Fi network, password is nkvvw35634 and then connect to whatever khepera you are working on, I normally work on Khepera 5 as that's the one given to us

Note: you need some sort of text editor for the following instructions, you can install vim using the following command:
$ sudo apt-get install vim

You will need to make a config file:
$ mkdir ~/.ssh
$ vim ~/.ssh/config

In the config file paste the following: 
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

To ssh into a robot (for our purposes, it's the khepera05):
$ ssh khepera05