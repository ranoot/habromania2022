motor_const = 0.17
axle_track = 0.175

#TODO: check if motors actually rotate at said speed
def d_steer_velo(s: float, r: float) -> tuple[float, float]:
	"""
	returns m_r, m_l
	"""
	r = r if r <= 1 else 1
	r = r if r >= -1 else -1
	m_r, m_l = 0, 0
	if r < 0:
		m_r = s - 2*s*abs(r)
		m_l = s
	if r > 0:
		m_l = s - 2*s*abs(r)
		m_r = s
	return (m_r, m_l)

def turning_radius(m_r, m_l):
	return (axle_track/2) * ((m_l + m_r)/(m_l - m_r))

def ang_velo(m_r, m_l):
	return ((m_l - m_r) * motor_const)/axle_track

def displacement_vector(s, r, delta_t) -> tuple[float, float, float]:
	"""
	assumes s > 0
	"""
	M = d_steer_velo(s, r)

	R = turning_radius(*M)
	omega = ang_velo(*M)

	theta = delta_t * omega
	x = R * (theta ** 2)/2
	y = abs(R) * abs(theta)

	return (x, y, theta)







