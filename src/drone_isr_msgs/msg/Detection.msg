# Bounding box coordinates (pixels)
int32 x_min
int32 y_min
int32 x_max
int32 y_max

# Classification
string label
float32 confidence

# Estimated world position (computed from altitude + pinhole geometry)
float32 world_x
float32 world_y
float32 world_z

# Timestamp of the source image
builtin_interfaces/Time stamp
