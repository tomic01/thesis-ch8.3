
string button_name	#Can be either text or a picture name. This picture
					#should be located in the folder /data/icons
					#of the screens project.
					#Usage: "text;ok"    "picture;yes_button.png"
					#(by default, it takes the text)
					
					
string topic_name	#Topic in which the button should publish when pressed.
					#The type of message should be std_msgs/String


string msg_value	#value of the message that is sent when the button is
					#pressed
