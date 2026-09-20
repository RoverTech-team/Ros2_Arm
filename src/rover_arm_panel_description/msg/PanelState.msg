std_msgs/Header header

# Switch positions in radians
float32 main_switch
float32[] lever_switches
float32[] rotary_switches
float32[] power_switches

# Indicator states (true = lit)
bool[] indicators

# Socket states (true = plugged)
bool[] sockets

# Electromagnet
bool electromagnet_on
bool plate_attached

# Task sequence tracking
bool task_main_switch_on
bool task_lever_switches_set
bool task_rotary_switches_set
bool task_plug_inserted
bool task_power_switch_1_on
bool task_power_switch_2_on
bool task_plate_placed
bool task_all_indicators_lit

# Damage tracking
bool[] elements_broken
string[] damage_info
