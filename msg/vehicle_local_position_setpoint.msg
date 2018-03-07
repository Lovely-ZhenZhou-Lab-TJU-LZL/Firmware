# Local position setpoint in NED frame

float32 x		# in meters NED
float32 y		# in meters NED
float32 z		# in meters NED
float32 yaw		# in radians NED -PI..+PI
float32 vx		# in meters/sec
float32 vy		# in meters/sec
float32 vz		# in meters/sec
float32 acc_x	# in meters/(sec*sec)
float32 acc_y	# in meters/(sec*sec)
float32 acc_z	# in meters/(sec*sec)

# added by lu
float32 vx_dot
float32 vy_dot
float32 vz_dot

float32 pos_x_err
float32 pos_y_err
float32 pos_z_err

float32 vel_x_err
float32 vel_y_err
float32 vel_z_err

bool in_traject_mode