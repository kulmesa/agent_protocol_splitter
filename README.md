<img align="right" height="20" src="https://auterion.com/wp-content/uploads/2020/05/auterion_logo_default_sunrise.svg">

# agent_protocol_splitter
[![GitHub](https://img.shields.io/github/license/Auterion/agent_protocol_splitter?style=for-the-badge)](https://github.com/Auterion/agent_protocol_splitter/blob/master/LICENSE)

microRTPS/DDS agent protocol splitter. Allows to stream and parse MAVLink and RTPS packets in the same serial link.

## Build

```sh
cmake ..
make
```

## Install

To install system wise on an UNIX system (might require `sudo` permissions):

```sh
make install
```

## Usage

Make sure to have the UART link up and the cabling right (if using an TTL-USB adapter).

Run the splitter using:

```sh
$ protocol splitter #TODO add options
```

At the same time, the `protocol_splitter` should also be run in the client side, so the stream can be multiplexed bidirectionally.
