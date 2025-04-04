import asyncio
from control.flight import FlightController
from control.utils import start_sim
from control.mission import square

async def main():
    # get dron object and init the flight controller
    drone = await start_sim()
    fc = FlightController(drone)

    # take off
    await fc.takeoff() # -> altitude of around 1.7m

    # simple mission from misson.py
    square_mission = square()
    for pos in square_mission:
        await fc.fly_to(pos)

    # go back to origin
    await fc.go_home()

    # land
    await fc.land()

    # disarming happens through PX4

if __name__ == "__main__":
    asyncio.run(main())