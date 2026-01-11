#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import csv
import os
import math


MAG_DECL = 0.01338  # magnetic_declination_radians

# ================================
# UTM MANUAL (NO EXTERNAL MODULE)
# ================================
def latlon_to_utm(lat, lon):
    # Constants
    a = 6378137.0           # WGS84 major axis
    f = 1 / 298.257223563   # flattening
    k0 = 0.9996

    e = math.sqrt(f * (2 - f))
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    zone_number = int((lon + 180) / 6) + 1
    lon0 = (zone_number - 1) * 6 - 180 + 3   # central meridian
    lon0_rad = math.radians(lon0)

    N = a / math.sqrt(1 - e*e * math.sin(lat_rad)**2)
    T = math.tan(lat_rad)**2
    C = (e*e / (1 - e*e)) * math.cos(lat_rad)**2
    A = math.cos(lat_rad) * (lon_rad - lon0_rad)

    M = a*((1 - e*e/4 - 3*e**4/64 - 5*e**6/256)*lat_rad
         - (3*e*e/8 + 3*e**4/32 + 45*e**6/1024)*math.sin(2*lat_rad)
         + (15*e**4/256 + 45*e**6/1024)*math.sin(4*lat_rad)
         - (35*e**6/3072)*math.sin(6*lat_rad))

    easting = k0 * N * (A + (1 - T + C)*A**3/6 
                       + (5 - 18*T + T*T + 72*C - 58*(e*e/(1-e*e))) * A**5/120) + 500000.0

    northing = k0 * (M + N * math.tan(lat_rad) *
                    (A*A/2 + (5 - T + 9*C + 4*C*C)*A**4/24
                     + (61 - 58*T + T*T + 600*C - 330*(e*e/(1-e*e))) * A**6/720))

    northern = (lat >= 0)
    if not northern:
        northing += 10000000.0

    return easting, northing, zone_number, northern


def utm_to_latlon(E, N, zone_number, northern):
    # Constants
    a = 6378137.0
    f = 1 / 298.257223563
    k0 = 0.9996
    e = math.sqrt(f * (2 - f))

    if not northern:
        N -= 10000000.0

    e1 = (1 - math.sqrt(1 - e*e)) / (1 + math.sqrt(1 - e*e))

    lon0 = (zone_number - 1) * 6 - 180 + 3
    M = N / k0

    mu = M / (a*(1 - e*e/4 - 3*e**4/64 - 5*e**6/256))

    phi1 = (mu
        + (3*e1/2 - 27*e1**3/32)*math.sin(2*mu)
        + (21*e1*e1/16 - 55*e1**4/32)*math.sin(4*mu)
        + (151*e1**3/96)*math.sin(6*mu))

    C1 = (e*e/(1-e*e)) * math.cos(phi1)**2
    T1 = math.tan(phi1)**2
    N1 = a / math.sqrt(1 - e*e * math.sin(phi1)**2)
    R1 = N1*(1 - e*e) / (1 - e*e*math.sin(phi1)**2)
    D = (E - 500000.0) / (N1 * k0)

    lat = (phi1 - (N1*math.tan(phi1)/R1) * 
        (D*D/2 - (5 + 3*T1 + 10*C1 - 4*C1*C1)*D**4/24 
         + (61 + 90*T1 + 298*C1 + 45*T1*T1 - 252*(e*e/(1-e*e)) - 3*C1*C1)*D**6/720))

    lon = math.radians(lon0) + (
        D - (1 + 2*T1 + C1)*D**3/6
        + (5 - 2*C1 + 28*T1 - 3*C1*C1 + 8*(e*e/(1-e*e)) + 24*T1*T1)*D**5/120
    ) / math.cos(phi1)

    return math.degrees(lat), math.degrees(lon)


# ============================================================
#                   GPS LOGGER NODE (NO MODULE)
# ============================================================
class GpsLogger(Node):
    def __init__(self):
        super().__init__('gps_logger')

        home = os.path.expanduser("~")
        self.csv_path = os.path.join(home, "gps_log.csv")

        self.f = open(self.csv_path, "w", newline='')
        self.writer = csv.writer(self.f)
        self.writer.writerow(["id", "latitude", "longitude"])

        self.id_counter = 1
        self.origin_set = False

        self.subscription_gps = self.create_subscription(
            NavSatFix, '/fix', self.gps_callback, 10
        )

        self.subscription_odom = self.create_subscription(
            Odometry, '/slam/odometry/filtered', self.odom_callback, 10
        )

        self.get_logger().info(f"Logging to {self.csv_path}")

    # ----------------------------------------------------------
    # FIRST GPS → SET UTM ORIGIN
    # ----------------------------------------------------------
    def gps_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude

        if not self.origin_set:
            E, N, zone, north = latlon_to_utm(lat, lon)
            self.origin_E = E
            self.origin_N = N
            self.origin_zone = zone
            self.origin_north = north
            self.origin_set = True
            self.get_logger().info(f"Origin set: lat={lat}, lon={lon}")

        # self.writer.writerow([self.id_counter, lat, lon])
        # self.id_counter += 1

    # ----------------------------------------------------------
    #   ODOM x,y → lat/lon menggunakan UTM + declination
    # ----------------------------------------------------------
    def odom_callback(self, msg):
        if not self.origin_set:
            return  # ignore until GPS origin known

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Koreksi magnetic declination
        xd = x * math.cos(MAG_DECL) - y * math.sin(MAG_DECL)
        yd = x * math.sin(MAG_DECL) + y * math.cos(MAG_DECL)

        # Tambahkan ke UTM origin
        E = self.origin_E + xd
        N = self.origin_N + yd

        # Konversi kembali UTM → lat/lon
        lat, lon = utm_to_latlon(E, N, self.origin_zone, self.origin_north)

        self.get_logger().info(f"Logged: lat={lat}, lon={lon}")

        self.writer.writerow([self.id_counter, lat, lon])
        self.id_counter += 1


# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = GpsLogger()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
