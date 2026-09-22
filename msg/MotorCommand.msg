# Fixed command shape for all controller modes.
# The mode chooses which field is active.
uint8 MODE_POSITION=0
uint8 MODE_VELOCITY=1
uint8 MODE_TORQUE=2

uint8 motor_id
uint8 mode
float64 position_rad
float64 velocity_rad_per_s
float64 kp
float64 kd
float64 torque_nm
