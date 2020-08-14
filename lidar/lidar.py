import VL53L1X


class Lidar:

    def __init__(self, i2c_address):

        # self.lidar = VL53L1X.VL53L1X(1, i2c_address)
        # self.lidar.open         ()
        # self.lidar.start_ranging(1)
        # Note:
        # 0 = Unchanged
        # 1 = Short Range
        # 2 = Medium Range
        # 3 = Long Range
        self.distance = 0

    def get_distance(self):

        # self.distance = self.lidar.get_distance()

        return self.distance

    def print_info(self, sensor_name):

        print('{}: next obstacle  @ {:6.2f} mm'.format(sensor_name, self.distance))