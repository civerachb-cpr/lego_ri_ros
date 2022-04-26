# The names of the joints being commanded
string[] name

# Revolute mode; restrict the motor position between an upper and lower bound
uint8 MODE_REVOLUTE=0

# Continuous mode; the motor can feely spin
uint8 MODE_CONTINUOUS=1

# One of the modes defined above
uint8[] drive_mode

# Used to set the stop mode of the motor
uint8 STOP_FLOAT=1
uint8 STOP_BRAKE=2
uint8 STOP_HOLD=3

# Ignored if zero/negative/not a valid mode from above
uint8[] stop_mode

# Upper and lower limits for the position of a motor
# operating in Revolute mode
float32[] upper_limit
float32[] lower_limit
