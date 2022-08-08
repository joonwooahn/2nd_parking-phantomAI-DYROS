
Header header           # Contains message publish time relative to publishing machine

# ESP Message (GW ID 0x706) - 10ms cycle time
float32 lat_accel        # -10.23 to 10.24 [m/s^2]. Positive = Counter Clockwise/Left
bool lat_accel_stat		 # 0 = lat_accel signal valid, 1 = lat_accel signal invalid
bool lat_accel_diag		 # 0 = lat_accel signal available, 1 = lat_accel signal not available

float32 long_accel       # -10.23 to 10.24 [m/s^2]
bool long_accel_stat     # 0 = long_accel signal valid, 1 = long_accel signal invalid
bool long_accel_diag     # 0 = long_accel signal available, 1 = long_accel signal not available

float32 yaw_rate         # -40.95 to 40.96 [deg/s'. Positive = Counter Clockwise
bool yaw_rate_stat		 # 0 = yaw_rate signal valid, 1 = yaw_rate signal invalid
bool yaw_rate_diag		 # 0 = yaw_rate signal available, 1 = yaw_rate signal not available

# Timestamp
float64 hw_timestamp    # timestamp of when the first CAN message was read by CAN HW