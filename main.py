import weakref
from math import sin
from math import cos
from math import radians
import carla
import random
from itertools import combinations


actor_list = []


def calculate_ego_vehicle_speed(az_1, el_1, radial_speed_1, az_2, el_2, radial_speed_2):
    a1 = cos(radians(az_1)) * cos(radians(el_1))
    b1 = sin(radians(az_1) * cos(radians(el_1)))
    a2 = cos(radians(az_2)) * cos(radians(el_2))
    b2 = sin(radians(az_2) * cos(radians(el_2)))

    c1 = radial_speed_1
    c2 = radial_speed_2

    v_x = (c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1)

    v_y = (a1 * c2 - a2 * c1) / (a1 * b2 - a2 * b1)

    ego_speed = (v_x, v_y)
    print(ego_speed)


try:
    # Connect to the client and retrieve the world object and blueprint library
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    bp = blueprint_library.filter("model3")[0]

    spawn_point = random.choice(world.get_map().get_spawn_points())

    vehicle = world.spawn_actor(bp, spawn_point)

    spectator = world.get_spectator()

    # Get the location and rotation of the spectator through its transform
    transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4, z=2.5)),
                                vehicle.get_transform().rotation)
    spectator.set_transform(transform)

    actor_list.append(vehicle)

    # Create the radar through a blueprint that defines its properties
    radar_bp = blueprint_library.filter('sensor.other.radar')[0]
    radar_bp.set_attribute('horizontal_fov', '30')
    radar_bp.set_attribute('vertical_fov', '30')
    radar_bp.set_attribute('points_per_second', '1500')
    radar_bp.set_attribute('range', '100')

    # Create a transform to place the radar on top of the vehicle
    radar_init_trans = carla.Transform(carla.Location(x=2.5, z=0.7))

    # We spawn the radar and attach it to our ego vehicle
    radar = world.spawn_actor(radar_bp, radar_init_trans, attach_to=vehicle)

    actor_list.append(radar)

    # Set vehicle in autopilot mode
    vehicle.set_autopilot(True)

    def is_valid_radar_data(data):
        if data.velocity < -500 or data.velocity > 0:
            return False
        else:
            return True

    # Create a radar's callback
    def radar_callback(weak_radar, sensor_data):

        self = weak_radar()
        if not self:
            return

        valid_sensor_data = filter(is_valid_radar_data, sensor_data)

        valid_sensor_data_list = list(valid_sensor_data)

        sensor_data_group = valid_sensor_data_list[0: 4]

        radar_data_combination_of_two_group = combinations(sensor_data_group, 2)

        for i in radar_data_combination_of_two_group:
            #print(f"az1 {i[0].azimuth}, el1 {i[0].altitude}, vel1 {i[0].velocity}, az2 {i[1].azimuth}, el2 {i[1].altitude}, vel2 {i[1].velocity}")
            calculate_ego_vehicle_speed(i[0].azimuth, i[0].altitude, i[0].velocity, i[1].azimuth, i[1].altitude,
                                        i[1].velocity)


    weak_radar = weakref.ref(radar)
    # Bind the callback
    radar.listen(lambda sensor_data: radar_callback(weak_radar, sensor_data))

finally:
    for actor in actor_list:
        actor.destroy()
        print("All clean up")
