#use standard header
Header header

string sentence
string emotion  
int32 volume
string agent                    # The voice of the robot

uint8 priority
uint8 SHUT_UP_IMEDIATLY = 0     # Abort current loqution. Clear queue
uint8 AFTER_CURRENT     = 1     # Wait current loqution to finish. Clear queue
uint8 APPEND            = 2     # Append to loqutions Queue