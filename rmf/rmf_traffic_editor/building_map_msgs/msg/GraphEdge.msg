uint32 v1_idx
uint32 v2_idx
Param[] params

# when edge_type is UNIDIRECTIONAL, it means v1 -> v2
# when edge_type is BIDIRECTIONAL, it means v1 <-> v2
uint8 edge_type
uint8 EDGE_TYPE_BIDIRECTIONAL=0
uint8 EDGE_TYPE_UNIDIRECTIONAL=1

# added by gmy - 28-Jul-2021, the tolerate number of AMRs which share a same lane
uint8 capacity
