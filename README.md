# sparki_explorer
Dead reckoning and object avoidance experiment

/******************************************* 

 Sparki explorer experiemnt
 
 
 Goal is to have Sparki explore a flat space
 
 with reasonable obstructions using a random
 
 walk and ultrasonic ping, and return to its
 
 origin or resume exploring upon command
 
 from the IR remote
 
********************************************/

IR Commands:


1 = Start Exploring

2 = Return to "home" (location where exploration started)

3 = Stop and set current location to "home"

UP = increase delay time (used for tuning loop behavior)

DOWN = decrease delay time (used for tuning loop behavior)

LEFT = decrease maximum range Sparki is allowed to explore (min = 40 cm)

RIGHT = increase maximum range Sparki is allowed to explore

MINUS = decrease the number of steps Sparki takes per loop when moving or rotating

PLUS = increase the number of steps Sparki takes per loop when moving or rotating




LED Status:


Green = at Home

Blue  = exploring

Red   = returning home
