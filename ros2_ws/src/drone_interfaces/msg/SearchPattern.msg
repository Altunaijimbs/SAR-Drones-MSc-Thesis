# Search pattern definition for SAR operations
std_msgs/Header header

# Pattern type: GRID, SPIRAL, EXPANDING_SQUARE, SECTOR, CUSTOM
string pattern_type

# Search area boundaries (polygon points or corners)
geometry_msgs/Point[] boundaries

# Pattern-specific parameters as key-value pairs
# For GRID: spacing, overlap_percentage, search_direction
# For SPIRAL: radius_increment, max_radius
# For EXPANDING_SQUARE: side_increment, max_side_length
string[] parameter_keys
float64[] parameter_values

# Search parameters
float32 altitude  # Search altitude in meters
float32 speed     # Search speed in m/s
bool return_to_start  # Return to start position after completion

# Mission metadata
string mission_id
string priority  # HIGH, MEDIUM, LOW