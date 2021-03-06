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

    distance_normalized = distance_from_center/(track_width/2.0)
    dist_award = 1-distance_normalized**4

    if (closest_waypoint>0 and closest_waypoint<len(waypoints)):
        print("closest_waypoint: %s" % (closest_waypoint))
        print("waypoints: %s" % (waypoints))
        closest_waypoint_point = waypoints[closest_waypoint]
        closest_waypoint_dist = ((closest_waypoint_point[0]-x)**2 + (closest_waypoint_point[1]-y)**2)**(0.5)
    else:
        closest_waypoint_dist = 100


    vel_discount = (1 - max(throttle, 0.4)) ** (1/max(closest_waypoint_dist, 0.1))

    print("distance_normalized: %s" % (distance_normalized))
    print("dist_award: %s" % (dist_award))
    print("vel_discount: %s" % (vel_discount))
    print("(1 - max(throttle, 0.4)): %s" % (1 - max(throttle, 0.4)))
    print("(1/max(closest_waypoint, 0.1)): %s" % (1/max(closest_waypoint, 0.1)))

    reward =  dist_award * vel_discount

    return reward
    


def test_center():
    reward_1 = reward_function(True, 1.0, 1.0, 0.5, 1.0, 1.0, 1.0, 0.99, 1.0, 1.0, [], 0)
    reward_2 = reward_function(True, 1.0, 1.0, 0.4, 1.0, 1.0, 1.0, 0.99, 1.0, 1.0, [], 0)
    print("reward_1: %s > reward_2: %s" % (reward_1, reward_2))
    assert(reward_1 < reward_2)

def test_lower_speed():  
    reward_1 = reward_function(True, 1.0, 1.0, 0.2, 1.0, 1.0, 1.0, 0.99, 1.0, 1.0, [[1,1,1], [2,2,2]], 2)
    reward_2 = reward_function(True, 1.0, 1.0, 0.2, 1.0, 1.0, 1.0, 0.99, 1.0, 1.0, [[1,1,1], [2,2,2]], 1)
    print("reward_1: %s > reward_2: %s" % (reward_1, reward_2))
    assert(reward_1 > reward_2)
