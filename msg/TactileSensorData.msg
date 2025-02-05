# Header
std_msgs/Header header

# Array of sensor normal forces in the site frame
float64[] forces

# Array of sensor positions in the site frame
geometry_msgs/Point[] positions

# Array of sensor normal directions in the site frame
geometry_msgs/Vector3[] normals

# Sensor surface type
uint8 surface_plane 1
uint8 surface_cylinder 2
int32 surface_type

# Sensor grid type
uint8 grid_square 1
uint8 grid_hex 2
int32 grid_type

# Sensor interval
float64 sensor_interval
