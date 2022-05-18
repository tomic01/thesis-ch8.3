#use standard header
Header header

string soundname
string action       # play_immediately, play_after_current, append, shut_up  (Default 'play_after_current')
uint8 volume        # 1 to 100                                  (Default '50')
uint8 repetitions   # Number of times to repeat the sound       (Default 0) 
uint32 fadein       # Time in milliseconds between repetitions  (Default 1000)
uint32 pitch        
uint32 duration     # duration in ms of the sound.