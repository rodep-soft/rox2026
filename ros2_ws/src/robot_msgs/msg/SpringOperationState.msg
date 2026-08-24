# Spring mechanism operation state for controller coordination.
uint8 IDLE = 0
uint8 HOMING = 1
uint8 NORMAL_FIRE = 2
uint8 SLOW_FIRE = 3
uint8 BELT_CLEARANCE = 4
uint8 ERROR = 5

uint8 state
