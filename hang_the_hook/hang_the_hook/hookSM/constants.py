# --- PID Gains ---
KP_X = 0.02
KI_X = 0.0
KD_X = 0.01

KP_Y = 0.02
KI_Y = 0.0
KD_Y = 0.01

KP_YAW = 0.02
KI_YAW = 0.0
KD_YAW = 0.01

# --- State timeout and Tolerances ---
TIMEOUT_ALIGN = 15.0   # Seconds
CENTER_TOLERANCE = 30  # Pixels
ANGULAR_TOLERANCE = 10 # Degrees
DROP_DIST = 1.8        # Meters

# --- Machine Outcomes ---
FIND_HOSE = 'find_hose'
DESCEND   = 'descend'
ALIGN     = 'align'
