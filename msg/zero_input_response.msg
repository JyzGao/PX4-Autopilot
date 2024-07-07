
uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32[6] dx          # Slots for zero input state derivatives, in the order of: du, dv, dw, dp, dq, dr
