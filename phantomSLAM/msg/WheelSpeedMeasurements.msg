
Header header           # Contains message publish time relative to publishing machine

# Wheel Speed Message (GW ID 0x720) - 20ms cycle time
float32 front_left      # 0 to 511.969 [km/h]
float32 front_right     # 0 to 511.969 [km/h]
float32 rear_left       # 0 to 511.969 [km/h]
float32 rear_right      # 0 to 511.969 [km/h]

# Timestamp
float64 hw_timestamp    # timestamp of when the CAN message was read by CAN Hardware.