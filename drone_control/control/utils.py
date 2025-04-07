from mavsdk import System
import asyncio

async def start_sim():
    drone = System() # get the PX4 drone
    await drone.connect(system_address="udp://:14540")  # default PX4 UDP port
    print("Waiting for drone...")
    async for state in drone.core.connection_state(): # go through all of the states
        if state.is_connected: # check the drone connection
            print("Drone discovered!")
            break

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("[INFO] GPS ready")
            break
        print("[WAITING] GPS not ready yet...")
        await asyncio.sleep(1)

    async for gps in drone.telemetry.gps_info():
        print(f"GPS: {gps}")
        break

    return drone