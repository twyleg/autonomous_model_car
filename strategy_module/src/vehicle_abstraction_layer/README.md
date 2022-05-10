# Vehicle Abstraction Layer

## Virtual CAN

To setup virtual can for testing purposes, load the can-gw kernel module:

	modprobe can-gw

TO setup two virtual can devices, run the following commands with root privileges:

	ip link add dev vcan0 type vcan
	ip link add dev vcan1 type vcan
	ip link set up vcan0
	ip link set up vcan1
	cangw -A -s vcan0 -d vcan1 -e
	cangw -A -s vcan1 -d vcan0 -e
