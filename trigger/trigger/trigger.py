"""
Node the interacts with an Arduino through serial communication.

This node is specifically designed for th eNerf Gun project, and
it is a bridge to ask the arduino to fire a Nerf gun.

Services (offered)
------------------
    - /fire (trigger_interfaces/srv/Fire) - Request a gun to be fired


"""


import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports as list_ports
from trigger_interfaces.srv import Fire


class Trigger(Node):
    def __init__(self):
        super().__init__("trigger")

        # we're identifying the Arudino by its serial number
        ARDUINO_SERIAL = "85439313130351612011"

        # ports discovery
        ports = list(list_ports.comports(True))  # get list of ports
        arduino_port = None

        # grab the port that's connected to the Arduino
        for p in ports:
            if p.serial_number == ARDUINO_SERIAL:
                arduino_port = p.device
                break

        # check Arduino has been found
        if arduino_port is None:
            print("error finding arduino")

        # create the serial connection
        self.serial_connection = serial.Serial(arduino_port)

        # create services
        self.service = self.create_service(Fire, "fire", self.fire_callback)

    def fire_callback(self, request, response):
        """
        Fire Nerf gun.

        Args:
        ----
        request: service request with the gun id
        response: returns an empty response when finished

        """
        gun_id = request.gun_id
        self.get_logger().info(f"Received request to fire gun with id {gun_id}")

        # send information to Arduino
        self.serial_connection.write(f"{gun_id}\n".encode(encoding="us-ascii"))

        # wait for Arduino response
        raw_line = self.serial_connection.readline()
        if raw_line:
            self.get_logger().info("Received confirmation from arduino")

        return response


def main(args=None):
    rclpy.init(args=args)
    trigger = Trigger()
    rclpy.spin(trigger)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
