/****************************************************************************
 *
 * Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <protocol_splitter.hpp>

#include <atomic>
#include <thread>
#include <algorithm>

std::mutex mtx;


/*********************************************************************
 * ByteBuffer
 *********************************************************************/

ByteBuffer::ByteBuffer()
	: buffer(0)
	, buffer_size(0)
	, pos(0)
	, len(0)
{

}

ByteBuffer::~ByteBuffer()
{
	if (buffer)
		std::free(buffer);
}

size_t ByteBuffer::resize(size_t n)
{
	assert(pos+n <= buffer_size);
	len = n;
	return len;
}

size_t ByteBuffer::reserve(size_t n)
{
	if (buffer_size >= n) {
		return buffer_size;
	}
	if (len > n) {
		// Existing data does not fit to new buffer
		return buffer_size;
	}

	uint8_t *old_buffer = buffer;
	buffer = (uint8_t*) std::malloc(n);
	if (buffer) {
		if (len) {
			memcpy(buffer, old_buffer+pos, len);
		}
		pos = 0;
		buffer_size = n;
	}

	if (old_buffer)
		std::free(old_buffer);

	if (!buffer)
	{
		printf("  reserve: malloc failed: %p\n", buffer);
		assert(0);
	}

	return buffer_size;
}

ssize_t ByteBuffer::push(void *dest, size_t n)
{
	ssize_t ret = reserve(pos+n);

	if (ret < pos+n) {
		return -1;
	}

	memcpy(buffer+pos, dest, n);
	len += n;
	return n;
}

ssize_t ByteBuffer::pop(void *dest, size_t n)
{
	assert(len >= n);
	assert(pos+n <= buffer_size);
	if (dest)
		memcpy(dest, buffer+pos, n);
	pos += n;
	len -= n;
	return n;
}


UartByteBuffer::UartByteBuffer(size_t size)
	: ByteBuffer()
{
	reserve(size);
}

ssize_t UartByteBuffer::fill()
{
	if (pos > BUFFER_FLUSH_THRESHOLD)
	{
		if (len > 0) {
			memmove(buffer, buffer+pos, len);
		}
		pos = 0;
	}
	DEBUG_UART_TO_UDP_PRINT(("UartByteBuffer::fill pos:%ld, len=%ld, _uart_fd:%d\n", pos, len, _uart_fd));
	ssize_t r = ::read(_uart_fd, write_point(), get_free() );
	if (r > 0)
		len += r;
	return r;
}


UdpByteBuffer::UdpByteBuffer(size_t size)
	: ByteBuffer()
{
	reserve(size);
}

ssize_t UdpByteBuffer::fill()
{
	if (pos > BUFFER_FLUSH_THRESHOLD)
	{
		if (len > 0) {
			memmove(buffer, buffer+pos, len);
		}
		pos = 0;
	}
	DEBUG_UDP_TO_UART_PRINT(("udp fill: free %ld\n", get_free()));
	static socklen_t addrlen = sizeof(_inaddr);
	ssize_t r = recvfrom(_udp_fd, write_point(), get_free(), 0, (struct sockaddr *) &_inaddr, &addrlen);
	if (r > 0)
		len += r;
	return r;
}


/*********************************************************************
 * DevSerial
 *********************************************************************/

DevSerial::DevSerial(const char *device_name, const uint32_t baudrate, const bool hw_flow_control,
		     const bool sw_flow_control)
	: _baudrate(baudrate)
	, _hw_flow_control(hw_flow_control)
	, _sw_flow_control(sw_flow_control)
	, _uart_read_buffer(BUFFER_SIZE)
{
	strncpy(_uart_name, device_name, sizeof(_uart_name));
	int fd = open_uart();
	if (fd >= 0)
		_uart_read_buffer.init(fd);
}

DevSerial::~DevSerial()
{
	if (_uart_fd >= 0) {
		close();
	}
}

ssize_t DevSerial::read()
{
	ssize_t r = _uart_read_buffer.fill();
	DEBUG_UART_TO_UDP_PRINT(("DevSerial::read len:%ld\n", r));
	if (r > 0)
		parse();

	return r;
}

ssize_t DevSerial::uart_write(ByteBuffer *vect)
{
	DEBUG_UDP_TO_UART_PRINT(("uart_write: _uart_fd %d\n", _uart_fd));
	if (-1 == _uart_fd) {
		return -1;
	}

	int ret = 0;
	DEBUG_UDP_TO_UART_PRINT(("uart_write: len: %ld\n", vect->size()));
	ret = ::write(_uart_fd, vect->data(), vect->size());
	vect->clear();
	return ret;
}

void DevSerial::parse()
{
	bool need_more_data = false;
	ssize_t mav_offset = STATUS_NOT_FOUND;
	ssize_t rtps_offset = STATUS_NOT_FOUND;
	DEBUG_UART_TO_UDP_PRINT(("DevSerial::parse... size:%ld\n", _uart_read_buffer.size()));
	while (_uart_read_buffer.size() && !need_more_data) {
		rtps_offset = objects->rtps->check_msgs(_uart_read_buffer, objects->rtps->get_uart_msg_handler());
		if (rtps_offset == STATUS_RECEIVING) {
			// RtpsDev in middle of message receiving..
			objects->rtps->send_msg(_uart_read_buffer, objects->rtps->get_uart_msg_handler());
			continue;
		}

		mav_offset = objects->mavlink2->check_msgs(_uart_read_buffer, objects->mavlink2->get_uart_msg_handler());
		if (mav_offset == STATUS_RECEIVING) {
			// Mavlink2Dev in middle of message receiving..
			objects->mavlink2->send_msg(_uart_read_buffer, objects->mavlink2->get_uart_msg_handler());
			continue;
		}

		if (rtps_offset < 0 && mav_offset < 0)
		{
			// No messages found.
			if (rtps_offset == STATUS_NEED_MORE_DATA || mav_offset == STATUS_NEED_MORE_DATA)
			{
				// Not enough data to verify message. Request to keep remaining
				// data in the receive buffer and fetch more to continue
				need_more_data = true;
			}
			else
			{
				// Buffer contains only unknown data, Remove all
				_uart_read_buffer.clear();
			}
		}
		else
		{
			// Buffer contains new message(s)
			if (rtps_offset >= 0)
			{
				// Buffer contains new RTPS message(s)
				if (mav_offset < 0)
				{
					// No Mavlink messages -> Only RTPS messages found
					objects->rtps->send_msg(_uart_read_buffer, objects->rtps->get_uart_msg_handler());
				}
				// Both RTPS and Mavlink messages found. Check which one is
				// earlier in the buffer and read it out
				else if (mav_offset < rtps_offset)
				{
					objects->mavlink2->send_msg(_uart_read_buffer, objects->mavlink2->get_uart_msg_handler());
				}
				else
				{
					objects->rtps->send_msg(_uart_read_buffer, objects->rtps->get_uart_msg_handler());
				}
			}
			else if (mav_offset >=0)
			{
				// No RTPS messages, but only new Mavlink message(s)
				objects->mavlink2->send_msg(_uart_read_buffer, objects->mavlink2->get_uart_msg_handler());
			}
		}

	} // while
}

void DevSerial::send_msg(ByteBuffer &buffer, MessageData &msg)
{
	uint8_t *data = buffer.data();
	DEBUG_UDP_TO_UART_PRINT(("UART: send_msg status: %ld\n", msg.status));
	if (msg.status == STATUS_RECEIVING)
	{
		size_t len = std::min(msg.remaining(), buffer.size());
		DEBUG_UDP_TO_UART_PRINT(("  UART: send_msg RECEIVING - remaining bytes: %ld, buffer.size: %ld\n", msg.remaining(), buffer.size()));
		buffer.pop(msg.data(), len);
		msg.update_receiving(len);
		DEBUG_UDP_TO_UART_PRINT(("  UART: send_msg RECEIVING - remaining bytes after pop: %ld, buffer.size after pop: %ld\n", msg.remaining(), buffer.size()));
		if (!msg.remaining())
		{
			uart_write(&msg.buffer);
			msg.clear();
		}
	}
	else
	{
		if (msg.status > 0)
		{
			// Extra rubbish in the beginning, remove
			buffer.pop(nullptr, msg.status);
			msg.status = 0;
		}
		DEBUG_UDP_TO_UART_PRINT(("  UART: send_msg -- packet_len: %ld\n", msg.packet_len));
		assert(msg.packet_len);
		size_t len = std::min(msg.remaining(), buffer.size());
		DEBUG_UDP_TO_UART_PRINT(("  UART: send_msg remaining bytes: %ld, packet_len: %ld, buffer.size(): %ld\n", msg.remaining(), msg.packet_len, buffer.size()));
		buffer.pop(msg.data(), len);
		msg.update_receiving(len);
		DEBUG_UDP_TO_UART_PRINT(("  UART: send_msg remaining after fetch: %ld, buffer.size(): %ld\n", msg.remaining(), buffer.size()));
		if (!msg.remaining()) {
			uart_write(&msg.buffer);
			msg.clear();
		} else {
			msg.status = STATUS_RECEIVING;
		}
	}
}

int DevSerial::open_uart()
{
	// Open a serial port, if not opened already
	if (_uart_fd < 0) {
		_uart_fd = open(_uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_uart_fd < 0) {
			printf("\033[0;31m[ protocol__splitter ]\tSerial link: Failed to open device: %s (%d)\033[0m\n", _uart_name, errno);
			return -errno;
		}

		// If using shared UART, no need to set it up
		if (_baudrate == 0) {
			return _uart_fd;
		}

		// Try to set baud rate
		struct termios uart_config;
		int termios_state;

		// Back up the original uart configuration to restore it after exit
		if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0) {
			int errno_bkp = errno;
			printf("\033[0;31m[ protocol__splitter ]\tSerial link: Error getting config %s: %d (%d)\033[0m\n", _uart_name,
			       termios_state, errno);
			close();
			return -errno_bkp;
		}

		// Set up the UART for non-canonical binary communication: 8 bits, 1 stop bit, no parity.
		uart_config.c_iflag &= !(INPCK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXANY | IXOFF);
		uart_config.c_iflag |= IGNBRK | IGNPAR;

		uart_config.c_oflag &= !(OPOST | ONLCR | OCRNL | ONOCR | ONLRET | OFILL | NLDLY | VTDLY);
		uart_config.c_oflag |= NL0 | VT0;

		uart_config.c_cflag &= !(CSIZE | CSTOPB | PARENB);
		uart_config.c_cflag |= CS8 | CREAD | CLOCAL;

		uart_config.c_lflag &= !(ISIG | ICANON | ECHO | TOSTOP | IEXTEN);

		// Flow control
		if (_hw_flow_control) {
			// HW flow control
			uart_config.c_lflag |= CRTSCTS;

		} else if (_sw_flow_control) {
			// SW flow control
			uart_config.c_lflag |= (IXON | IXOFF | IXANY);
		}

		// Set baud rate
		speed_t speed;

		if (!baudrate_to_speed(_baudrate, &speed)) {
			printf("\033[0;31m[ protocol__splitter ]\tSerial link: Error setting baudrate %s: Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 921600, 1000000\033[0m\n",
			       _uart_name, _baudrate);
			close();
			return -EINVAL;
		}

		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			int errno_bkp = errno;
			printf("\033[0;31m[ protocol__splitter ]\tSerial link: Error setting baudrate %s: %d (%d)\033[0m\n", _uart_name,
			       termios_state, errno);
			close();
			return -errno_bkp;
		}

		if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
			int errno_bkp = errno;
			printf("\033[0;31m[ protocol__splitter ]\tSerial link: ERR SET CONF %s (%d)\033[0m\n", _uart_name, errno);
			close();
			return -errno_bkp;
		}

		printf("[ protocol__splitter ]\tSerial link: device: %s; baudrate: %d; flow_control: %s\n",
		       _uart_name, _baudrate, _sw_flow_control ? "SW enabled" : (_hw_flow_control ? "HW enabled" : "No"));

		char aux[64];
		bool flush = false;

		while (0 < ::read(_uart_fd, (void *)&aux, 64)) {
			flush = true;
			usleep(1000);
		}

		if (flush) {
			printf("[ protocol__splitter ]\tSerial link: Flush\n");

		} else {
			printf("[ protocol__splitter ]\tSerial link: No flush\n");
		}
	}

	return _uart_fd;
}

bool DevSerial::baudrate_to_speed(uint32_t bauds, speed_t *speed)
{
#ifndef B460800
#define B460800 460800
#endif

#ifndef B500000
#define B500000 500000
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
#endif

#ifndef B1500000
#define B1500000 1500000
#endif

#ifndef B2000000
#define B2000000 2000000
#endif

	switch (bauds) {
	case 0:      *speed = B0;		break;

	case 50:     *speed = B50;		break;

	case 75:     *speed = B75;		break;

	case 110:    *speed = B110;		break;

	case 134:    *speed = B134;		break;

	case 150:    *speed = B150;		break;

	case 200:    *speed = B200;		break;

	case 300:    *speed = B300;		break;

	case 600:    *speed = B600;		break;

	case 1200:   *speed = B1200;		break;

	case 1800:   *speed = B1800;		break;

	case 2400:   *speed = B2400;		break;

	case 4800:   *speed = B4800;		break;

	case 9600:   *speed = B9600;		break;

	case 19200:  *speed = B19200;		break;

	case 38400:  *speed = B38400;		break;

	case 57600:  *speed = B57600;		break;

	case 115200: *speed = B115200;		break;

	case 230400: *speed = B230400;		break;

	case 460800: *speed = B460800;		break;

	case 500000: *speed = B500000;		break;

	case 921600: *speed = B921600;		break;

	case 1000000: *speed = B1000000;	break;

	case 1500000: *speed = B1500000;	break;

	case 2000000: *speed = B2000000;	break;

#ifdef B3000000

	case 3000000: *speed = B3000000;    break;
#endif

#ifdef B3500000

	case 3500000: *speed = B3500000;    break;
#endif

#ifdef B4000000

	case 4000000: *speed = B4000000;    break;
#endif

	default:
		return false;
	}

	return true;
}

int DevSerial::close()
{
	if (_uart_fd >= 0) {
		printf("\033[1;33m[ protocol__splitter ]\tSerial link: Closed serial port!\033[0m\n");
		::close(_uart_fd);
		_uart_fd = -1;
	}

	return 0;
}




/*********************************************************************
 * DevSocket
 *********************************************************************/

DevSocket::DevSocket(const char *udp_ip,
                     const uint16_t udp_port_recv,
                     const uint16_t udp_port_send)
	: _udp_port_recv(udp_port_recv)
	, _udp_port_send(udp_port_send)
	, _udp_read_buffer(BUFFER_SIZE)
{
	if (nullptr != udp_ip) {
		strcpy(_udp_ip, udp_ip);
	}

	if (!open_udp())
		_udp_read_buffer.init(_udp_fd, &_inaddr);
}

DevSocket::~DevSocket()
{
	// Close socket
	close();
}

int DevSocket::open_udp()
{
	// Init receiver
	if ((_udp_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		printf("\033[0;31m[ protocol__splitter ]\tUDP socket link: Create socket failed\033[0m\n");
		return -1;
	}

	memset((char *)&_inaddr, 0, sizeof(_inaddr));
	_inaddr.sin_family = AF_INET;
	_inaddr.sin_port = htons(_udp_port_recv);
	_inaddr.sin_addr.s_addr = htonl(INADDR_ANY);

	memset((char *) &_outaddr, 0, sizeof(_outaddr));
	_outaddr.sin_family = AF_INET;
	_outaddr.sin_port = htons(_udp_port_send);
	_outaddr.sin_addr.s_addr = htonl(INADDR_ANY);

	printf("[ protocol__splitter ]\tUDP socket link: Trying to connect...\n");

	if (bind(_udp_fd, (struct sockaddr *)&_inaddr, sizeof(_inaddr)) < 0) {
		printf("\033[0;31m[ protocol__splitter ]\tUDP socket link: Bind failed\033[0m\n");
		return -1;
	}

	printf("[ protocol__splitter ]\tUDP socket link: Connected to server!\n");

	if (inet_aton(_udp_ip, &_outaddr.sin_addr) == 0) {
		printf("\033[0;31m[ protocol__splitter ]\tUDP socket link: inet_aton() failed\033[0m\n");
		return -1;
	}

	return 0;
}

ssize_t DevSocket::udp_read()
{
	if (-1 == _udp_fd) {
		return -1;
	}
	static socklen_t addrlen = sizeof(_inaddr);
	ssize_t r = _udp_read_buffer.fill();
	DEBUG_UDP_TO_UART_PRINT(("udp_read %ld\n", r));
	return r;
}

ssize_t DevSocket::udp_write(ByteBuffer *vect)
{
	if (-1 == _udp_fd) {
		return -1;
	}

	DEBUG_UART_TO_UDP_PRINT(("udp_write: size: %ld\n", vect->size()));
	int ret = sendto(_udp_fd, vect->data(), vect->size(), 0, (struct sockaddr *)&_outaddr, sizeof(_outaddr));
	vect->clear();
	return ret;
}

void DevSocket::send_msg(ByteBuffer &buffer, MessageData &msg)
{
	uint8_t *data = buffer.data();
	if (msg.status == STATUS_RECEIVING)
	{
		size_t len = std::min(msg.remaining(), buffer.size());
		DEBUG_UART_TO_UDP_PRINT(("  UDP: send_msg RECEIVING - remaining bytes: %ld, buffer.size: %ld\n", msg.remaining(), buffer.size()));
		buffer.pop(msg.data(), len);
		DEBUG_UART_TO_UDP_PRINT(("  UDP: send_msg -- RECEIVING -- emaining bytes after fetch: %ld\n", msg.remaining()));
		msg.update_receiving(len);
		if (!msg.remaining())
		{
			udp_write(&msg.buffer);
			msg.clear();
		}
	}
	else
	{
		if (msg.status > 0)
		{
			// Extra rubbish in the beginning, remove
			buffer.pop(nullptr, msg.status);
			msg.status = 0;
		}
		DEBUG_UART_TO_UDP_PRINT(("  UDP: send_msg -- packet_len: %ld\n", msg.packet_len));
		assert(msg.packet_len);
		size_t len = std::min(msg.remaining(), buffer.size());
		DEBUG_UART_TO_UDP_PRINT(("  UDP: send_msg remaining bytes: %ld, packet_len: %ld, buffer.size(): %ld\n", msg.remaining(), msg.packet_len, buffer.size()));
		buffer.pop(msg.data(), len);
		msg.update_receiving(len);
		DEBUG_UART_TO_UDP_PRINT(("  UDP: send_msg remaining after fetch: %ld, buffer.size(): %ld\n", msg.remaining(), buffer.size()));
		if (!msg.remaining()) {
			udp_write(&msg.buffer);
			msg.clear();
		} else {
			msg.status = STATUS_RECEIVING;
		}
	}
}

void DevSocket::parse()
{
	bool need_more_data = false;
	ssize_t offset = STATUS_NOT_FOUND;
	DEBUG_UDP_TO_UART_PRINT(("DevSocket::parse... size:%ld\n", _udp_read_buffer.size()));
	while (_udp_read_buffer.size() && !need_more_data) {
		offset = check_msgs(_udp_read_buffer, get_udp_msg_handler());
		DEBUG_UDP_TO_UART_PRINT(("check_msgs: status: %ld\n", offset));
		if (offset == STATUS_NEED_MORE_DATA)
		{
			// Not enough data to verify message. Request to keep remaining
			// data in the receive buffer and fetch more to continue
			need_more_data = true;
		}
		else if (offset == STATUS_NOT_FOUND)
		{
			// No messages found. Buffer contains only unknown data, Remove all
			_udp_read_buffer.clear();
		}
		else
		{
			DEBUG_UDP_TO_UART_PRINT(("send_msg..\n"));
			// Valid messages found, send to UART
			objects->serial->send_msg(_udp_read_buffer, get_udp_msg_handler());
		}

	} // while
}

ssize_t DevSocket::read()
{
	ssize_t r = udp_read();
	if (r > 0)
		parse();

	return r;
}

int DevSocket::close()
{
	if (_udp_fd >= 0) {
		printf("\033[1;33m[ protocol__splitter ]\tUDP socket link: Closed socket!\033[0m\n");
		shutdown(_udp_fd, SHUT_RDWR);
		::close(_udp_fd);
		_udp_fd = -1;
	}

	return 0;
}

/*********************************************************************
 * Mavlink2Dev
 *********************************************************************/

Mavlink2Dev::Mavlink2Dev(const char *udp_ip,
                         const uint16_t udp_port_recv,
                         const uint16_t udp_port_send)
	: DevSocket(udp_ip, udp_port_recv, udp_port_send)
{
	_udp_msg.clear();
	_uart_msg.clear();
}

ssize_t Mavlink2Dev::check_msgs(ByteBuffer &buffer, MessageData &msg)
{
	uint8_t *data = buffer.data();
	size_t size = buffer.size();

	// Ignore check in case in middle of receiving..
	if (msg.status == STATUS_RECEIVING)
		return msg.status;

	msg.clear();
	for (size_t pos = 0; pos < size; pos++)
	{
		if (data[pos] == 254)
		{
			DEBUG_PRINT(("Mavlink2Dev::check_msgs: magic-254 found!\n"));
			// Mavlink1 message
			if (pos+3 >= size)
			{
				msg.status = STATUS_NEED_MORE_DATA;
				break;
			}
			else
			{
				msg.init_packet(data[pos+1]+8);
				msg.status = pos;
				break;
			}
		}
		else if (data[pos] == 253)
		{
			// Mavlink2 message
			if (pos+7 >= size)
			{
				msg.status = STATUS_NEED_MORE_DATA;
				break;
			}
			else
			{
				size_t packet_len = data[pos+1] + 12;
				uint8_t incompat_flags = data[pos+2];
				DEBUG_UDP_TO_UART_PRINT(("Mavlink2 seq: %d\n", data[pos+4]));
				if (incompat_flags & 1) {
					packet_len += 13;
				}
				msg.init_packet(packet_len);
				msg.status = pos;
				break;
			}
		}
	}
	return msg.status;
}


/*********************************************************************
 * RtpsDev
 *********************************************************************/

RtpsDev::RtpsDev(const char *udp_ip,
                 const uint16_t udp_port_recv,
                 const uint16_t udp_port_send)
	: DevSocket(udp_ip, udp_port_recv, udp_port_send)
{
	_udp_msg.clear();
	_uart_msg.clear();
}

ssize_t RtpsDev::check_msgs(ByteBuffer &buffer, MessageData &msg)
{
	uint8_t *data = buffer.data();
	size_t size = buffer.size();

	// Ignore check in case in middle of receiving..
	if (msg.status == STATUS_RECEIVING)
		return msg.status;

	msg.status = STATUS_NOT_FOUND;
	msg.clear();

	for (size_t pos = 0; pos < size; pos++)
	{
		if (data[pos] == '>')
		{
			// RTPS message
			if (pos+7 >= size)
			{
				msg.status = STATUS_NEED_MORE_DATA;
				break;
			}
			else if (data[pos+1] == '>' && data[pos+2] == '>')
			{
				msg.init_packet( (data[pos+5] << 8 | data[pos+6]) + 9);
				msg.status = pos;
				break;
			}
		}
	}
	return msg.status;
}


/*********************************************************************
 * Poll & Signal handlers
 *********************************************************************/

void signal_handler(int signum)
{
	printf("\033[1;33m[ protocol__splitter ]\tInterrupt signal (%d) received.\033[0m\n", signum);
	running = false;
}

void serial_receiver(pollfd *fds)
{
	while (running) {
		if ((::poll(fds, sizeof(fds) / sizeof(fds[0]), 100) > 0) && (fds[0].revents & POLLIN)) {
			//DEBUG_UART_TO_UDP_PRINT(("serial_receiver\n"));
			objects->serial->read();
		}
	}
}

void mavlink_udp_to_serial(pollfd *fds)
{
	while (running) {
		if ((::poll(fds, sizeof(fds) / sizeof(fds[0]), 100) > 0) && (fds[0].revents & POLLIN)) {
			//DEBUG_UDP_TO_UART_PRINT(("mavlink_udp_to_serial\n"));
			objects->mavlink2->read();
		}
	}
}

void rtps_udp_to_serial(pollfd *fds)
{
	while (running) {
		if ((::poll(fds, sizeof(fds) / sizeof(fds[0]), 100) > 0) && (fds[0].revents & POLLIN)) {
			//DEBUG_UDP_TO_UART_PRINT(("rtps_udp_to_serial\n"));
			objects->rtps->read();
		}
	}
}


/*********************************************************************
 * Main
 *********************************************************************/

static void usage(const char *name)
{
	printf("usage: %s [options]\n\n"
	       "  -b <baudrate>			UART device baudrate. Default 460800\n"
	       "  -d <uart_device>		UART device. Default /dev/ttyUSB0\n"
	       "  -i <host_ip>			Host IP for UDP. Default 127.0.0.1\n"
	       "  -w <mavlink_udp_recv_port>	UDP port for receiving. Default 5800\n"
	       "  -x <mavlink_udp_send_port>	UDP port for receiving. Default 5801\n"
	       "  -y <rtps_udp_recv_port>	UDP port for receiving. Default 5900\n"
	       "  -z <rtps_udp_send_port>	UDP port for receiving. Default 5901\n"
	       "  -f <sw_flow_control>		Activates UART link SW flow control\n"
	       "  -h <hw_flow_control>		Activates UART link HW flow control\n"
	       "  -v <verbose_debug>		Add more verbosity\n\n",
	       name);
}

static int parse_options(int argc, char **argv)
{
	int ch;

	while ((ch = getopt(argc, argv, "b:d:i:w:x:y:z:fghv")) != EOF) {
		switch (ch) {
		case 'b': _options.baudrate		  = strtoul(optarg, nullptr, 10);		  break;

		case 'd': if (nullptr != optarg)	strcpy(_options.uart_device, optarg); break;

		case 'i': if (nullptr != optarg)	strcpy(_options.host_ip, optarg);  	  break;

		case 'f': _options.sw_flow_control = true;								  break;

		case 'g': _options.hw_flow_control = true;								  break;

		case 'h': usage(argv[0]); return -1;									  break;

		case 'v': _options.verbose_debug = true;								  break;

		case 'w': _options.mavlink_udp_recv_port  = strtoul(optarg, nullptr, 10); break;

		case 'x': _options.mavlink_udp_send_port  = strtoul(optarg, nullptr, 10); break;

		case 'y': _options.rtps_udp_recv_port     = strtoul(optarg, nullptr, 10); break;

		case 'z': _options.rtps_udp_send_port     = strtoul(optarg, nullptr, 10); break;

		default:
			usage(argv[0]);
			return -1;
		}
	}

	if (_options.hw_flow_control && _options.sw_flow_control) {
		printf("\033[0;31m[ protocol__splitter ]\tHW and SW flow control set. Please set only one or another\033[0m\n");
		return -1;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	if (-1 == parse_options(argc, argv)) {
		return -1;
	}

	objects = new StaticData();

	std::signal(SIGINT, signal_handler);

	// Init the serial device
	objects->serial = new DevSerial(
		_options.uart_device,
		_options.baudrate,
		_options.hw_flow_control,
		_options.sw_flow_control);

	// Init UDP sockets for Mavlink and RTPS
	objects->mavlink2 = new Mavlink2Dev(
		_options.host_ip,
		_options.mavlink_udp_recv_port,
		_options.mavlink_udp_send_port);

	objects->rtps = new RtpsDev(
		_options.host_ip,
		_options.rtps_udp_recv_port,
		_options.rtps_udp_send_port);

	// Init fd polling
	pollfd fd_uart[1]{};
	pollfd fds_udp_mavlink[1]{};
	pollfd fds_udp_rtps[1]{};

	fd_uart[0].fd = objects->serial->get_fd();
	fd_uart[0].events = POLLIN;

	fds_udp_mavlink[0].fd = objects->mavlink2->get_fd();
	fds_udp_mavlink[0].events = POLLIN;

	fds_udp_rtps[0].fd = objects->rtps->get_fd();
	fds_udp_rtps[0].events = POLLIN;

	running = true;

	std::thread serial_receiver_th(serial_receiver, fd_uart);
	std::thread rtps_udp_to_serial_th(rtps_udp_to_serial, fds_udp_rtps);
	std::thread mavlink_udp_to_serial_th(mavlink_udp_to_serial, fds_udp_mavlink);

	serial_receiver_th.join();
	mavlink_udp_to_serial_th.join();
	rtps_udp_to_serial_th.join();

	delete objects->serial;
	delete objects->mavlink2;
	delete objects->rtps;
	delete objects;
	objects = nullptr;

	printf("\033[1;33m[ protocol__splitter ]\tEXITING...\033[0m\n");

	return 0;
}
