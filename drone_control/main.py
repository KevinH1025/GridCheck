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

    # THIS DOES NOT WORK, NOT SURE WHY
    # simple mission from misson.py
    # square_mission = square()[:-1]
    # for pos in square_mission:
    #    await fc.GPS_fly_to(pos)

    # simple mission from misson.py
    square_mission = square()
    for pos in square_mission:
        await fc.NED_fly_to(pos)

    # go back to origin
    await fc.go_home()

    # disarming happens through PX4

if __name__ == "__main__":
    asyncio.run(main())