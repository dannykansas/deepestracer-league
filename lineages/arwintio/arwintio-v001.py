def reward_function(on_track, x, y, distance_from_center, car_orientation, progress, steps, throttle, steering, track_width, waypoints, closest_waypoint):

    '''
    @on_track (boolean) :: The vehicle is off-track if the front of the vehicle is outside of the white
    lines

    @x (float range: [0, 1]) :: Fraction of where the car is along the x-axis. 1 indicates
    max 'x' value in the coordinate system.

    @y (float range: [0, 1]) :: Fraction of where the car is along the y-axis. 1 indicates
    max 'y' value in the coordinate system.

    @distance_from_center (float [0, track_width/2]) :: Displacement from the center line of the track
    as defined by way points

    @car_orientation (float: [-3.14, 3.14]) :: yaw of the car with respect to the car's x-axis in
    radians

    @progress (float: [0,1]) :: % of track complete

    @steps (int) :: numbers of steps completed
⁄
    @throttle :: (float) 0 to 1 (0 indicates stop, 1 max throttle)

    @steering :: (float) -1 to 1 (-1 is right, 1 is left)

    @track_width (float) :: width of the track (> 0)

    @waypoints (ordered list) :: list of waypoint in order; each waypoint is a set of coordinates
    (x,y,yaw) that define a turning point

    @closest_waypoint (int) :: index of the closest waypoint (0-indexed) given the car's x,y
    position as measured by the eucliedean distance

    @@output: @reward (float [-1e5, 1e5])
    '''

    import math
    # Example Centerline following reward function

    reward = 0
    if not on_track:
        reward += (float(-4e4) * float(progress)) - 100

    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width
    marker_4 = 0.75 * track_width

    if distance_from_center >= 0.0 and distance_from_center <= marker_1:
        reward += 1.5
    elif distance_from_center <= marker_2:
        reward += 0.5
    elif distance_from_center <= marker_3:
        reward += 0.1
    elif distance_from_center <= marker_4:
        reward -= 0.2
    else:
        reward += -1

    THROTTLE_THRESHOLD = 0.8
    if throttle < THROTTLE_THRESHOLD:
        reward -= (2 - throttle) * 5
    elif throttle > 0.95:
        reward += throttle * 10
    else:
        reward += throttle * 3

    ABS_STEERING_THRESHOLD = 0.75
    if abs(steering) > ABS_STEERING_THRESHOLD:
        reward -= (1.5 - throttle) * 3
    else:
        reward += 0.75

    closest_waypoint_wp = waypoints[closest_waypoint]
    next_waypoint = waypoints[(closest_waypoint+1) % len(waypoints)]

    angle_between_waypoints = math.atan2(
        next_waypoint[1] - y,
        next_waypoint[0] - x,
    )
    diff_between_car_angle_and_waypoint_line = abs(
        angle_between_waypoints - car_orientation
    )
    reward += 3.14 - diff_between_car_angle_and_waypoint_line
    if diff_between_car_angle_and_waypoint_line > 0.8:
        reward -= 3

    distance_between_car_and_next_waypoint = math.sqrt(
        math.pow(next_waypoint[0] - x, 2) +
        math.pow(next_waypoint[1] - y, 2)
    )
    reward += (1 - distance_between_car_and_next_waypoint) * 3

    reward += progress

    return float(reward)
