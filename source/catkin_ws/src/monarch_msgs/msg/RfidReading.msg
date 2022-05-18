#use standard header
Header header

uint64  tag_id	     			  #tag id value
string  tag_type	 			  #type: 'robot'(localization),'person'(detection)
string  tag_info     		      #tag description
float64 tag_prob		     	  #probability value
geometry_msgs/Point tag_position  #tag position estimation
float64 tag_angle    			  #tag angle, for people detection
