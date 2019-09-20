# Header: info of the message. The frame_id is used for uniquely 
#	identifying a Line_Segment.
# Line_List: if true, use a LINE_LIST if false a LINE_STRIP.
# 	The LINE_STRIP type uses each point as a vertex
# 	in a connected set of lines, where point 0 is 
# 	connected to point 1, 1 to 2, 2 to 3, etc. The 
# 	LINE_LIST type creates unconnected lines out of 
# 	each pair of points, i.e. point 0 to 1, 2 to 3, etc.
# points: list of indexed points according to the line_list value
# color: color to be used, such as Gazebo/blue, Gazebo/purple, etc.
# action: one of three actions as following
#	ACTION_ADD_MODIFY: add or modify a previous Line_Segment
#	ACTION_REMOVE: remove a specific Line_Segment
#	ACTION_REMOVE_ALL: remove all Line_Segments

Header header
bool line_list 
geometry_msgs/Point32[] points
string color
float32[] risk
uint8 action
uint8 ACTION_ADD_MODIFY=0
uint8 ACTION_REMOVE=1
uint8 ACTION_REMOVE_ALL=2