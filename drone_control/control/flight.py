import asyncio
import math
from mavsdk import System
from mavsdk.offboard import PositionNedYaw
from mavsdk.telemetry import PositionNed, FlightMode

class FlightController():
    def __init__(self, drone:System):
        self.drone = drone

    # take off function
    async def takeoff(self):
        print("Arming the drone...")
        await self.drone.action.arm()
        print("Drone Armed.")

        print("Taking off...")
        await self.drone.action.takeoff() # default: drone climbs to 2.5m

        async for position in self.drone.telemetry.position():
            if position.relative_altitude_m >= 1.7: # margin for error
                print(f"Drone reached stable altitude: {position.relative_altitude_m}")
                break
    
    # landing function
    async def land(self):
        # turn off offboard
        await self.drone.offboard.stop()

        # switch to Auto land
        await self.drone.action.land()
        print("Landing...")

        # is drone still flying?
        async for in_air in self.drone.telemetry.in_air(): 
            if not in_air:
                print("Drone landed")
                break
    
    # fly in NED coordinates -> more accurate
    async def NED_fly_to(self, target_pos:tuple):
        print(f"moving to: North = {target_pos[0]}, East = {target_pos[1]}, Down = {target_pos[2]}")

        # to go into off board it needs to get good estimation of local position
        async for health in self.drone.telemetry.health():
            if health.is_local_position_ok:
                break
        
        # set coordinates before switching to offboard mode -> it needs constant update of the coordinates, ref. point = armed location
        target_ned = PositionNedYaw(*target_pos)
        await self.drone.offboard.set_position_ned(target_ned)

        # offboard mode very senstitive hence, try/except block
        try:
            await self.drone.offboard.start()
            await self.drone.offboard.set_position_ned(target_ned)
        except Exception as e:
            print(f"Offboard failed: {e}")
            await self.land()
        
        # loop to send the offboard singnals repeatedly 
        while True:
            await self.drone.offboard.set_position_ned(target_ned)

            # get current distance to the desired point
            current_pos = await self.get_pos()
            distance = self.compute_distance(current_pos, target_ned)

            # position counted reached within this margin
            if distance <= 0.2:
                break
            
            # ~10Hz update rate
            await asyncio.sleep(0.1)  

    # fly in GPS coordinates -> less accurate -> IT DOES NOT WORK, I DO NOT KNOW WHY
    async def GPS_fly_to(self, target:tuple):
        # get current postion in deg 
        async for position in self.drone.telemetry.position():
            break

        # conversion meters in degree
        delta_degs = self.distance_to_deg(target, position.latitude_deg)

        # update the coordinates
        target_lat = position.latitude_deg + delta_degs[0]
        target_lon = position.longitude_deg + delta_degs[1]
        target_alt = position.absolute_altitude_m + target[2]

        # go to this new location
        await self.drone.action.goto_location(latitude_deg = target_lat,
                                              longitude_deg = target_lon,
                                              absolute_altitude_m = target_alt,
                                              yaw_deg = 0
                                              )
        
        # print to debug
        print(f"moving to: lat = {target_lat}, lon = {target_lon}, alt = {target_alt}")

        # wait until it reaches 
        async for position in self.drone.telemetry.position():
            dist_lat = abs(target_lat - position.latitude_deg) * 111320
            dist_lon = abs(target_lon - position.longitude_deg) * 111320 * math.cos(math.radians(position.latitude_deg))
            dist_alt = abs(target_alt - position.absolute_altitude_m)
            if dist_lat < 0.5 and dist_lon < 0.5 and dist_alt < 0.5:
                print(f"reached: lat = {target_lat}, lon = {target_lon}, alt = {target_alt}")
                break

    # go home function
    async def go_home(self):
        async for mode in self.drone.telemetry.flight_mode():
            break

        if mode is FlightMode.OFFBOARD:
            current_pos:PositionNed = await self.get_pos()
            await self.NED_fly_to((0, 0, current_pos.down_m, 0))
            # land
            await self.land()
        else:
            self.drone.action.set_return_to_launch_altitude(10)
            self.drone.action.return_to_launch()
            print("returning home...")
            async for in_air in self.drone.telemetry.in_air(): 
                if not in_air:
                    print("Drone landed")
                    break

    #------------------------------------------------------
    # helper function to get current position of the drone
    async def get_pos(self):
        async for current in self.drone.telemetry.position_velocity_ned():
                current_pos = current.position
                break
        return current_pos

    # helper function to calculate L2 distance
    @staticmethod
    def compute_distance(current:PositionNed, target:PositionNedYaw):
        return math.sqrt(
            (current.north_m - target.north_m)**2 + 
            (current.east_m - target.east_m)**2 + 
            (current.down_m - target.down_m)**2
        )
    
    # helper function for converstion, but not accurate only for testing. TODO: WGS84 ellipsoid model
    @staticmethod
    def distance_to_deg(target, lat0):
        delta_lat = target[0] / 111320.0
        delta_lon = target[1] / (111320.0 * math.cos(math.radians(lat0)))
        delta_degs = (delta_lat, delta_lon)
        return delta_degs