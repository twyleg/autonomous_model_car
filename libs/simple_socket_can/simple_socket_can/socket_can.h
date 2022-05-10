// Copyright (C) 2021 twyleg
#pragma once

#include <linux/can.h>

#include <string>

class SocketCAN {

public:

	SocketCAN(const std::string& interfaceName);

	int write(const can_frame&);
	int read(can_frame&);

	void setBlocking(bool blocking) const;
	size_t dataAvailable() const;

private:

	const std::string mInterfaceName;
	const int mSock;

};
