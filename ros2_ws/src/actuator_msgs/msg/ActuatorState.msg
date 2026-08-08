# ActuatorState.msg

uint8 STATE_OFFLINE=0
uint8 STATE_INITIALIZING=1
uint8 STATE_READY=2
uint8 STATE_ERROR=3

uint16 logical_id
uint8 state

bool position_reference_set

float32 position
float32 velocity
float32 torque_nm
float32 temperature

uint32 fault_code