uint64 PC_time_usec                 # Receive timestamp in autopilot
float32[2] t                        # Start time and stop time
float32[7] trajectory_coefficient_x # X-axis trajectory coefficient
float32[7] trajectory_coefficient_y # Y-axis trajectory coefficient
float32[7] trajectory_coefficient_z # Z-axis trajectory coefficient
float32[7] trajectory_coefficient_r # YAW-axis trajectory coefficient
uint8 num_keyframe                  # Sum of keyframe
uint8 index_keyframe                # Index of keyframe
uint8 order_p_1                     # Order plus one
