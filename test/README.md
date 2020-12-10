# Test protocol_splitter

To run tests you need to have PX4 SITL simulation running<br>
Start PX4 SITL simulation from px4-firmware project: https://github.com/PX4/PX4-Autopilot<br>
Clone the px4-Autopilot and start simulation for drone, e.g.:
```
make px4_sitl_rtps gazebo_iris_rtps
```

### Run test scripts
1. Run test_protocol_splitter.sh script to start virtual com ports and protocol_splitter instances
```
./test_protocol_splitter.sh
```

2. In new terminal shell, run mav_reader.py
```
./mav_reader.py
```
You should see mavlink messages coming from px4

3. In new terminal shell, run rtps_reader.py
```
./rtps_reader.py
```
You should see rtps messages coming from px4
