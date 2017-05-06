uint32 id			# ID of the target

uint64 timestamp_received	# timestamp when the estimate was received
float32 dt				# delta time
float32[4] q			# Estimated attitude as quaternion
float32[4] body_q		# attitude of quad
float32 x			# X position in meters in NED earth-fixed frame
float32 y			# Y position in meters in NED earth-fixed frame
float32 z			# Z position in meters in NED earth-fixed frame (negative altitude)
