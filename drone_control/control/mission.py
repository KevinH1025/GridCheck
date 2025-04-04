from mavsdk.offboard import PositionNedYaw

# simple mission -> draw a square
def square():
    return [(10, 10, -10, 0), # north, east, down, yaw
            (-10, 10, -10, 0),
            (-10, -10, -10, 0),
            (10, -10, -10, 0),
            (10, 10, -10, 0)
    ]