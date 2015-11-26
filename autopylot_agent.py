#!/usr/bin/python

'''
Python code for the autopilot of the AR Drone. When the autopilot is active the drone will detect and
track faces. A facial database can be added so familiar faces can be recognized. If no faces are 
detected the drone will slowly yaw.

Modified code originaly provided by Simon D. Levy

John Kirby
'''
import numpy as np
from face_tracker_V1 import track


# PID parameters
Kpx = 0.04
Kpy = 0.25
Kpt = 0.12
Kpr = 0.13
Kdx = 0.04
Kdy = 0.25
Kdt = 0.16
Kdr = 0.13
Kix = 0
Kiy = 0
Kit = 0
Kir = 0

# Target square size for neutral pitch
WIDTH_TARGET = 100
WIDTH_FACTOR = 0.003
YAW_FACTOR = 1.6


# Routine called by C program.
def action(img_bytes, img_width, img_height, is_belly, is_auto, ctrl_state, vbat_flying_percentage, theta, phi, psi, altitude, vx, vy):

    #print(is_auto)

    # Set up command defaults
    zap = 0
    phi = 0     
    theta = 0 
    gaz = -0.008
    yaw = .05
    

    # Set up state variables first time around
    if not hasattr(action, 'count'):
        action.count = 0
        action.errx_1 = 0
        action.erry_1 = 0
	action.errd_1 = 0
	action.errR_1 = 0
        action.phi_1 = 0
        action.gaz_1 = 0
	action.theta_1 = 0
	action.yaw_1 = 0

    # Create full-color image from bytes
    nparr = np.frombuffer(img_bytes, np.uint8)
    nparr = np.ndarray.reshape(nparr, (img_height,img_width,3))
                    
    # Grab track the center of the face of the person
    target = track(nparr, vbat_flying_percentage, is_auto)

    # Use centroid if it exists
    if not target is None:

        # Target contains center and width of square
        ctr, w = target

	#Stop the drone rotation
	yaw = 0
	gaz = 0 	

        # Compute proportional distance (error) of centroid from image center
        errx =  _dst(ctr, 0, img_width)
        erry = -_dst(ctr, 1, img_height)

	# Compute proportional distance (error) of face from proper distance
	#errd = (WIDTH_TARGET - w)*WIDTH_FACTOR
	errd = (w - WIDTH_TARGET)*WIDTH_FACTOR
	errR = errx*YAW_FACTOR       

        # Compute vertical, horizontal, and forward velocity commands based on PID control after first iteration
        if action.count > 0:
            phi = _pid(action.phi_1, errx, action.errx_1, Kpx, Kix, Kdx)
            gaz = _pid(action.gaz_1, erry, action.erry_1, Kpy, Kiy, Kdy)
	    theta = _pid(action.theta_1, errd, action.errd_1, Kpt, Kit, Kdt)
	    yaw = _pid(action.yaw_1, errR, action.errR_1, Kpr, Kir, Kdr)
	    
        # Remember PID variables for next iteration
        action.errx_1 = errx
        action.erry_1 = erry
	action.errd_1 = errd
	action.errR_1 = errR
        action.phi_1 = phi
        action.gaz_1 = gaz
	action.theta_1 = theta
	action.yaw_1 = yaw

    # Keep a count for debugging
    action.count += 1
    
    # Send control parameters back to drone
    return (zap, phi, theta, gaz, yaw)


# Simple PID controller from http://www.control.com/thread/1026159301
def _pid(out_1, err, err_1, Kp, Ki, Kd):
    return Kp*err + Ki*(err+err_1) + Kd*(err-err_1)
    
 

# Returns proportional distance to image center along specified dimension.
# Above center = -; Below = +
# Right of center = +; Left = -
def _dst(ctr, dim, siz):
    siz = siz/2
    return (ctr[dim] - siz) / float(siz)
