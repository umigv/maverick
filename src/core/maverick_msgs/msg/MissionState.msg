bool in_recovery             # recovery behavior active; goal selection paused
bool in_ramp_approach        # within ramp_approach_radius of a ramp_approach waypoint; speed capped
bool in_no_mans_land         # inside a no man's land zone; extra inflation active
bool lane_detection_enabled  # false inside no man's land except near zone exit
bool mission_complete        # final waypoint reached; all navigation should stop

geometry_msgs/PointStamped current_waypoint  # current target waypoint, in map frame
