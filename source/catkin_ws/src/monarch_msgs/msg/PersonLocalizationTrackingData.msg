#use standard header
Header header

uint32 id                              # tracker id
geometry_msgs/PoseWithCovariance pose  # tracked person estimated pose
uint32[] groupFellows                  # Who is this person in a group with
geometry_msgs/Point vel                # Velocity of the person in x,y,z

int32 ACT_WALK=0                       # if isActive[ACT_WALK]==true, the person is detected as walking
int32 ACT_RUN=1                        # Running
int32 ACT_CRAFT=2                      # The person is doing handwork at a table
int32 ACT_TV=3                         # The person is watching TV
int32 NUM_ACT=4                        # Length of the isActive array. The idea is that code can check whether any activity is
                                       #    being detected without updating the code as new activities are added to the list.
bool[4] isActive                       # SIZE MUST BE UPDATED AS NUM_ACT IS UPDATED
                                       # What activities are discovered for this person 
