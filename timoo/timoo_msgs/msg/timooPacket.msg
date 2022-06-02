# Raw timoo LIDAR packet.

time stamp              # packet timestamp
uint8[1206] data        # packet contents
uint32 data_stamp       # packet timestamp[year-month-day-hour form DISOP]
bool gps_status         # gps status from DISOP