#use standard header
Header header

int32 id                     # id of person in front of robot. For now, this is not linked to the ids assigned by the overhead cameras
float32 person_orientation    # relative orientation of the person with respect to robot (0 degrees if person looking at robot, 180 degrees if facing away)
float32 direction_to_person   # relative orientation of the person with respect to robot (0 degrees if straight in front of the head of the robot).
float32 distance              # relative distance from the robot to the person
