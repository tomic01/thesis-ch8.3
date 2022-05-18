string name                  # ID of behavior, as defined by the constants above
uint32 instance_id		     # ID of specific instance of the behavior being requested
int8[] robots                # the numbers here correspond to mbotX. At least one robot should be selected.
KeyValuePair[] parameters    # parameters needed by the behavior, given as key/value pairs
string[] resources           # robot resources needed for execution of behavior. Taken from list of Functionalities (see Joyful Warden Technical Script): RNAV,RNAVFC,RArmMotion,...
bool active		     # indicates the execution/cancelation status of the request behavior (true = behavior to be execute, false = behavior to be canceled)