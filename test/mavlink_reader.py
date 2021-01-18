#!/bin/python3

import argparse
import socket
import serial
import sys

SERIAL_PORT = '/dev/vcom_px4'
UDP_ADDR = '127.0.0.1'
UDP_PORT = 4801

MAVLINK1_HEADER_LEN = 6
MAVLINK1_CHECKSUM_LEN = 2
MAVLINK2_HEADER_LEN = 10
MAVLINK2_CHECKSUM_LEN = 2
MAVLINK2_SIGNATURE_LEN = 13


STATUS_NEED_MORE_DATA = -3;
STATUS_NOT_FOUND = -2;
STATUS_RECEIVING = -1;

mavlink_packet_len = 0
parse_status = STATUS_NOT_FOUND
storage = b''
msg = b''

def decode_bytes(data):
	string = ''
	for byte in data:
		string += '{:02x} '.format(byte)

	print(' bytes: ', string)
	print()


def check_msgs():
	global mavlink_packet_len
	global parse_status
	global storage

	if parse_status == STATUS_RECEIVING:
		return

	parse_status = STATUS_NOT_FOUND
	pos = 0
	mavlink_packet_len = 0
	while pos < len(storage):
		if storage[pos] == 0xFD:
			parse_status = STATUS_NEED_MORE_DATA
			if len(storage) >= 3:
				parse_status = pos
				payload_len = storage[pos+1]
				mavlink_packet_len = MAVLINK2_HEADER_LEN + payload_len + MAVLINK2_CHECKSUM_LEN
				incomp_flg = storage[pos+2]
				if incomp_flg & 1:
					mavlink_packet_len += MAVLINK2_SIGNATURE_LEN
			return

		elif storage[pos] == 0xFE:
			parse_status = STATUS_NEED_MORE_DATA
			if len(storage) > MAVLINK1_HEADER_LEN:
				payload_len = storage[pos+1]
				parse_status = pos
				mavlink_packet_len = MAVLINK1_HEADER_LEN + payload_len + MAVLINK1_CHECKSUM_LEN
			return
		else:
			pos += 1

def read_message():
	global parse_status
	global storage
	global msg

	offset = 0
	if parse_status > 0:
		storage = storage[parse_status:]
	if (len(msg) + len(storage)) < mavlink_packet_len:
		msg += storage
		parse_status = STATUS_RECEIVING
	else:
		remaining = mavlink_packet_len - len(msg)
		if remaining < 0:
			print("ERROR: msg is bigger than packet len!!!")
		msg += storage[:remaining]
		storage = storage[remaining:]
		decode_mavlink(msg)
		msg = b''
		parse_status = STATUS_NOT_FOUND


def decode_mavlink(data):

	if data[0] == 0xFE:
		print()
		print(' Mavlink1')
		payload_len = data[1]
		print(' [0] Magic       : ' + hex(data[0]))
		print(' [1] Payload len : ' + str(data[1]))
		print(' [2] Sequence    : ' + str(data[2]))
		print(' [3] SysId       : ' + str(data[3]))
		print(' [4] CompId      : ' + str(data[4]))
		print(' [5] MsgId       : ' + str(data[5]))
		offset = MAVLINK1_HEADER_LEN
		pload = ' [n] Payload     : [ '
		for i in range(payload_len):
			pload += '{:02x} '.format(data[6+i])
		pload += ']'
		print(pload)
		offset += payload_len
		print('     Checksum    : ' + hex(data[offset] | data[offset+1]<<8))
		offset += MAVLINK1_CHECKSUM_LEN

	elif data[0] == 0xFD:
		print()
		print(' Mavlink2')
		payload_len = data[1]
		incomp_flg = data[2]
		print(' [0]   Magic       : ' + hex(data[0]))
		print(' [1]   Payload len : ' + str(data[1]))
		print(' [2]   IncompatFlg : ' + hex(data[2]))
		print(' [3]   CompatFlg   : ' + hex(data[3]))
		print(' [4]   Sequence    : ' + str(data[4]))
		print(' [5]   SysId       : ' + str(data[5]))
		print(' [6]   CompId      : ' + str(data[6]))
		print(' [7-9] MsgId       : ' + hex(data[7] | data[8] << 8 | data[9] << 16))
		offset = MAVLINK2_HEADER_LEN
		pload = ' [n] Payload     : [ '
		for i in range(payload_len):
			pload += '{:02x} '.format(data[6+i])
		pload += ']'
		print(pload)
		offset += payload_len
		print('     Checksum    : ' + hex(data[offset] | data[offset+1]<<8))
		offset += MAVLINK2_CHECKSUM_LEN
		if incomp_flg & 1:
			# signature exists in the message
			pload = '     Signature   : [ '
			for i in range(MAVLINK2_SIGNATURE_LEN):
				pload += '{:02x} '.format(data[offset+6+i])
			pload += ']'
			print(pload)
			offset += MAVLINK2_SIGNATURE_LEN

	return offset


def main():
	global parse_status
	global storage
	global msg
	parse_status = 0
	parser = argparse.ArgumentParser(description='Test protocol splitter')
	parser.add_argument('--serial', action='store', default="",
		help='Serial device to listen')
	parser.add_argument('--ip', action='store', default=UDP_ADDR,
		help='UDP address')
	parser.add_argument('--port', action='store', default=UDP_PORT,
		help='UDP port')

	args = parser.parse_args()

	source = None
	if args.serial != '':
		source = serial.Serial(args.serial, timeout=0.1)
		print('Listening to:', args.serial)
	else:
		source = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		addr = (args.ip, int(args.port))
		source.bind(addr)
		print('Listening to:', args.ip, ':', args.port)

	storage = b''
	msg = b''
	while True:
		data = b''
		if args.serial != '':
			while not len(data):
				data = source.read(256)
		else:
			data, address = source.recvfrom(256)
		print(' received', str(len(data)), 'bytes')
		decode_bytes(data)
		storage += data
		check_msgs()
		if parse_status == STATUS_NOT_FOUND:
			storage = b''
		elif parse_status != STATUS_NEED_MORE_DATA:
			read_message()

if __name__ == '__main__':
	main()
