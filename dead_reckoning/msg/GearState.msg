
Header header           # Contains message publish time relative to publishing machine

# Gear Selection (GW ID 0x700) - 10ms cycle time

uint8 GEAR_PARK = 0
uint8 GEAR_LOW = 1
uint8 GEAR_SECOND = 2
uint8 GEAR_THIRD = 3
uint8 GEAR_DS_MODE = 4
uint8 GEAR_DRIVE = 5
uint8 GEAR_NEUTRAL = 6
uint8 GEAR_REVERSE = 7
uint8 GEAR_SPORTS_MANUAL_PADDLE_MODE = 8
uint8 GEAR_NO_CLUSTER_DISPLAY = 9
uint8 GEAR_SUB_ROM_COM = 10
uint8 GEAR_SUB_ROM_COM_ERROR = 11
uint8 GEAR_PADDLE_SHIFT_DRIVE = 12
uint8 GEAR_RESERVED = 13
uint8 GEAR_INTERMEDIATE_POS = 14
uint8 GEAR_FAULT = 15

uint8 gear_state        # 0 = Park, 1 = Low, 2 = 2nd, 3 = 3rd, 4 = DS Mode, 5 = Drive, 6 = Neutral,
                        # 7 = Reverse, 8 = Sports mode/Manual shift/Paddle shift, 9 = Not Display at Cluster,
                        # 10 = Sub-ROM Com, 11 = Sub-ROM Com error, 12 = Paddle Shift (D-position), 13 = Reserved,
                        # 14 = Intermediate Position, 15 = Fault.  
                        
# Timestamp
float64 hw_timestamp    # timestamp of when the first CAN message was read by CAN HW 