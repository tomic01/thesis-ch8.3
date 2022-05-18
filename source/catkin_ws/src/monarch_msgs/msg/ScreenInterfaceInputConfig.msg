#A message used to configure the screen interface: if the input is 
#enabled or not, the number of buttons it should display and the
#configuration of each button.

#use standard header
Header header

bool enable_input				# false: hide menu; true: is required to show the menu

int16 number_buttons			#currently, only from 1 to 4 supported

ScreenButton[] buttons_config
