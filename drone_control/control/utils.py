from mavsdk import System

async def start_sim():
    drone = System() # get the PX4 drone
    await drone.connect(system_address="udp://:14540")  # default PX4 UDP port
    print("Waiting for drone...")
    async for state in drone.core.connection_state(): # go through all of the states
        if state.is_connected: # check the drone connection
            print("Drone discovered!")
            break

    return drone