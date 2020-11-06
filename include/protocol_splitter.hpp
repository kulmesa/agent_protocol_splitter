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

#include <arpa/inet.h>
#include <cassert>
#include <csignal>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

//#define __DEBUG__

#define BUFFER_SIZE 2048
#define BUFFER_FLUSH_THRESHOLD 1024
#define DEFAULT_BAUDRATE 460800
#define DEFAULT_UART_DEVICE "/dev/ttyUSB0"
#define DEFAULT_HOST_IP "127.0.0.1"
#define DEFAULT_MAVLINK_RECV_PORT 5800
#define DEFAULT_MAVLINK_SEND_PORT 5801
#define DEFAULT_RTPS_RECV_PORT 5900
#define DEFAULT_RTPS_SEND_PORT 5901


#ifdef __DEBUG__
	#define __DEBUG_UART_TO_UDP__
	#define __DEBUG_UDP_TO_UART__
	#define __DEBUG_COMMON__
#endif

#ifdef __DEBUG_COMMON__
	#define DEBUG_PRINT(pargs)    printf pargs
#else
	#define DEBUG_PRINT(pargs)
#endif

#ifdef __DEBUG_UART_TO_UDP__
	#define DEBUG_UART_TO_UDP_PRINT(pargs)    printf pargs
#else
	#define DEBUG_UART_TO_UDP_PRINT(pargs)
#endif
#ifdef __DEBUG_UDP_TO_UART__
	#define DEBUG_UDP_TO_UART_PRINT(pargs)    printf pargs
#else
	#define DEBUG_UDP_TO_UART_PRINT(pargs)
#endif


class DevSerial;
class Mavlink2Dev;
class RtpsDev;
class ByteBuffer;

struct StaticData {
	DevSerial *serial;
	Mavlink2Dev *mavlink2;
	RtpsDev *rtps;
};

volatile sig_atomic_t running = true;

struct options {
	uint32_t baudrate = DEFAULT_BAUDRATE;
	char uart_device[64] = DEFAULT_UART_DEVICE;
	char host_ip[16] = DEFAULT_HOST_IP;
	uint16_t mavlink_udp_recv_port = DEFAULT_MAVLINK_RECV_PORT;
	uint16_t mavlink_udp_send_port = DEFAULT_MAVLINK_SEND_PORT;
	uint16_t rtps_udp_recv_port = DEFAULT_RTPS_RECV_PORT;
	uint16_t rtps_udp_send_port = DEFAULT_RTPS_SEND_PORT;
	bool sw_flow_control = false;
	bool hw_flow_control = false;
	bool verbose_debug = false;
} _options;

namespace
{
const ssize_t STATUS_NEED_MORE_DATA = -3;
const ssize_t STATUS_NOT_FOUND = -2;
const ssize_t STATUS_RECEIVING = -1;
/*
-3 = STATUS_NEED_MORE_DATA    : Partial message header found, need more data to verify
-2 = STATUS_NOT_FOUND         : No messages found from data set
-1 = STATUS_RECEIVING         : Message receiving continues from previous round
<offset>                   : Message found from data set. offset from beginning
                             of the buffer.
*/
static StaticData *objects = nullptr;

}


class ByteBuffer
{
public:
	ByteBuffer();
	virtual ~ByteBuffer();

	size_t reserve(size_t n);
	size_t resize(size_t n);
	ssize_t push(void *dest, size_t n);
	ssize_t pop(void *dest, size_t n);
	void clear() { pos = 0; len = 0; }
	size_t size() { return len; }
	uint8_t* data() {
		assert(pos <= buffer_size);
		return buffer+pos; }
	size_t get_free() { return buffer_size-(pos+len); }

protected:
	uint8_t *buffer;
	size_t buffer_size = 0;
	ssize_t pos = 0;
	ssize_t len = 0;

	uint8_t* write_point() { return buffer+pos+len; }
};


class UartByteBuffer : public ByteBuffer
{
public:
	UartByteBuffer(size_t size);
	ssize_t fill();
	void init(int fd) { _uart_fd = fd; }
private:
	int _uart_fd = -1;
};


class UdpByteBuffer : public ByteBuffer
{
public:
	UdpByteBuffer(size_t size);
	ssize_t fill();
	void init(int fd, struct sockaddr_in *addr) { _udp_fd = fd; _outaddr = addr; }
private:
	int _udp_fd = -1;
	struct sockaddr_in *_outaddr = nullptr;
};


class MessageData
{
public:
	MessageData()
	{
		buffer.reserve(1024);
	}

	void init_packet(size_t len) {
		packet_len = len;
		buffer.clear();
		buffer.reserve(len);
		buffer.resize(len);
	}

	void clear() {
		status = STATUS_NOT_FOUND;
		packet_len = 0;
		index = 0;
		buffer.clear();
	}

	void update_receiving(size_t len) {
		index += len;
	}

	uint8_t* data() {
		assert(index < packet_len);
		return buffer.data() + index;
	}
	size_t size() {
		return index;
	}
	size_t remaining() {
		if (packet_len > index)
			return packet_len - index;
		else
			return 0;
	}

	ssize_t status;
	size_t  packet_len;
	ByteBuffer buffer;
	size_t  index;

};

class DevSerial
{
public:
	DevSerial(const char *device_name,
	          const uint32_t baudrate,
	          const bool hw_flow_control,
	          const bool sw_flow_control);
	virtual ~DevSerial();

	int	open_uart();
	ssize_t    read();
	void	   write();
	void send_msg(ByteBuffer &buffer, MessageData &msg);
	int        get_fd() { return _uart_fd; }

protected:

	uint32_t _baudrate;
	bool _hw_flow_control;
	bool _sw_flow_control;
	int _uart_fd = -1;

	char _uart_name[64] = {};
	UartByteBuffer _uart_read_buffer;

	ssize_t uart_write(ByteBuffer *vect);

	void parse();
	bool baudrate_to_speed(uint32_t bauds, speed_t *speed);
	int close();
private:

};

class DevSocket
{
public:
	DevSocket(const char *udp_ip,
	          const uint16_t udp_port_recv,
	          const uint16_t udp_port_send,
	          const bool server);
	virtual ~DevSocket();

	int close();

	int	open_udp();
	void send_msg(ByteBuffer &buffer, MessageData &msg);
	ssize_t    read();
	void       write();
	int        get_fd() { return _udp_fd; }

protected:
	virtual ssize_t check_msgs(ByteBuffer &buffer, MessageData &msg) = 0;
	virtual MessageData& get_udp_msg_handler() = 0;;

protected:
	char _udp_ip[16] = {};
	int _udp_fd;

	uint16_t _udp_port_recv;
	uint16_t _udp_port_send;
	struct sockaddr_in _outaddr;
	struct sockaddr_in _inaddr;
	UdpByteBuffer _udp_read_buffer;
	bool _server;

	ssize_t _msg_parse_status = STATUS_NOT_FOUND;
	size_t  _msg_packet_len = 0;

	//ByteBuffer *_msg_receive_buffer = nullptr;
	size_t  _msg_receive_index = 0;

	void parse();
	ssize_t udp_read();
	ssize_t udp_write(ByteBuffer *vect);

private:

};

class Mavlink2Dev : public DevSocket
{
public:
	Mavlink2Dev(const char *udp_ip,
	            const uint16_t udp_port_recv,
	            const uint16_t udp_port_send);
	virtual ~Mavlink2Dev() {}

	ssize_t check_msgs(ByteBuffer &buffer, MessageData &msg);
	MessageData& get_uart_msg_handler() { return _uart_msg; }

protected:
	MessageData& get_udp_msg_handler() { return _udp_msg; }
	MessageData _udp_msg;
	MessageData _uart_msg;
};

class RtpsDev : public DevSocket
{
public:
	RtpsDev(const char *udp_ip,
	        const uint16_t udp_port_recv,
	        const uint16_t udp_port_send);
	virtual ~RtpsDev() {}

	ssize_t check_msgs(ByteBuffer &buffer, MessageData &msg);
	MessageData& get_uart_msg_handler() { return _uart_msg; }

protected:
	MessageData& get_udp_msg_handler() { return _udp_msg; }
	MessageData _udp_msg;
	MessageData _uart_msg;
};
