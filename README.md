# icreate_cartpole

#Raspi setup:

The raspi username is pi, and password is raspberry (i.e. the defaults). It runs an SSH server and can be remotely accessed over SSH on port 22 using the command:
'''
ssh pi@<raspi ip>
'''
You can x-forward if you want, but no guarantees what will work and what won't.

The eth0 network interface is set (by modification of /etc/network/interfaces) to adopt a fixed ip of 10.66.171.21, and subnet mask of 255.255.240.0. So, to connect to this, you need to tell the network interface connecting to the raspi via direct ethernet link that subnet and assign it an IP within that subnet (I use 10.66.171.20).

Reference on here: https://pihw.wordpress.com/guides/direct-network-connection/. The Raspi side of things has been done slightly differently but to the same effect.

The Raspi is currently set up to connect to Izatt_guest, Izatt_guest_ext automatically on startup using network device wlan0. Because I don't supply a gateway for the eth0 interface, it'll default to using the gateway discovered via wlan0 if network connection is successful; the take home here is that if a wlan connection is made the raspi will still know how to connect to the Internet. On some startups this connection fails; in this case either manually force a connection to those networks (using iwconfig, probably), or reboot the Raspi and hope it gets it the next time around.
 