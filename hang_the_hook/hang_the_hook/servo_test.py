from nectar.control import(
    DroneFactory,
    MavlinkConfig,
)

def servo_test(drone) -> None:

    drone.do_servo(
        aux_out= 7,
        pwm_value= 1500
    )

def main() -> None:

    drone = DroneFactory.create(drone_type='mavlink', config=MavlinkConfig(connection_string="/dev/ttyAMA1"))

    servo_test(drone=drone)

if __name__ == '__main__':
    main()
