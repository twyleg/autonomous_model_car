// Copyright (C) 2021 twyleg
#include "socket_can.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <asm/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>

SocketCAN::SocketCAN(const std::string& interfaceName)
	: mInterfaceName(interfaceName),
	  mSock(socket(PF_CAN, SOCK_RAW, CAN_RAW))
{
	struct ifreq ifr;
	strcpy(ifr.ifr_name, mInterfaceName.c_str());
	ioctl(mSock, SIOCGIFINDEX, &ifr);

	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	bind(mSock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr) );
}

int SocketCAN::write(const can_frame& canFrame) {
	return ::write(mSock, &canFrame, sizeof(can_frame));
}

int SocketCAN::read(can_frame& canFrame) {
	return ::read(mSock, &canFrame, sizeof(can_frame));
}

void SocketCAN::setBlocking(bool blocking) const {
	const int flags = fcntl(mSock, F_GETFL);
	if (blocking){
		fcntl(mSock, F_SETFL, flags & ~O_NONBLOCK);
	}else{
		fcntl(mSock, F_SETFL, flags | O_NONBLOCK);
	}
}

size_t SocketCAN::dataAvailable() const {
	size_t dataAvailable;
	ioctl(mSock, FIONREAD, &dataAvailable);
	return dataAvailable;
}
