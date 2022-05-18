#use standard header
Header header

uint8 device         # index of the device to control
# 0 - LeftEyeLeds
# 1 - RightEyeLeds
# 2 - CheeksLeds
# 3 - BaseRightLeds
# 4 - BaseFrontLeds
# 5 - BaseLeftLeds
uint8 r              #red intensity percentage (0-100)
uint8 g              #green intensity percentage
uint8 b              #blue intensity percentage
float32 change_time  #time in seconds to reach intensity