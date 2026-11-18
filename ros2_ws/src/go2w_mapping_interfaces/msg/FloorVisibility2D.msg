# Actual horizontal floor returns, in odom_fused at acquisition time.
# Every endpoint is measured; missing directions imply no visibility.
std_msgs/Header header
float32 origin_x
float32 origin_y
float32[] endpoints_x
float32[] endpoints_y
