#!/usr/bin/env python3

import heapq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import csv
import os
import glob
import re
import random

import sys
from loguru import logger

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from tf2_ros import Buffer, TransformListener

# ============================
#          CONSTANTS
# ============================
METER_PER_DEGREE = 111319.4907932736
THRESH_PT_OVERLAP = 10.0
DOT_MARKER_SIZE = 2     
ROBOT_MARKER_SIZE = 8
# ============================

class LatLonNavigator:
    def __init__(self, segments, route):
        self.segments = segments
        self.route = route
        self.route_idx = 0

        # active segment
        self.active_idx = route[0]
        self.seg = self.segments[self.active_idx]

        # robot coordinate
        self.robot_lat = self.seg["lat"][0]
        self.robot_lon = self.seg["lon"][0]

        self.ptr = 1
        # self.speed = 0.0002    # lat/lon speed
        self.speed = 0.01    # lat/lon speed
        self._done = False

        # Figure + Animation
        self.fig, self.ax = plt.subplots(figsize=(7, 9))
        self.anim = FuncAnimation(self.fig, self.update, interval=60, cache_frame_data=False)
        plt.show()

        # SAVE timer for fix
        self._timer = self.anim.event_source

    # --------------------------------------------------
    def move_towards(self, lat_t, lon_t):
        dlat = lat_t - self.robot_lat
        dlon = lon_t - self.robot_lon
        dist = np.sqrt(dlat*dlat + dlon*dlon)

        if dist < 1e-12:
            return True

        step = min(self.speed, dist)
        self.robot_lat += (dlat / dist) * step
        self.robot_lon += (dlon / dist) * step
        return (step >= dist - 1e-12)

    # --------------------------------------------------
    def update(self, frame):
        if self._done:
            return

        self.ax.clear()

        # Draw all segments
        for idx, seg in enumerate(self.segments):
            c = "gray"
            if idx in self.route:
                c = "blue"
            if idx == self.active_idx:
                c = "red"
            self.ax.plot(seg["lon"], seg["lat"], "-o", color=c, markersize=DOT_MARKER_SIZE)

        # Current segment & target
        target_lat = self.seg["lat"][self.ptr]
        target_lon = self.seg["lon"][self.ptr]

        reached = self.move_towards(target_lat, target_lon)

        if reached:
            # next waypoint
            if self.ptr < len(self.seg["lat"]) - 1:
                self.ptr += 1
            else:
                # end of this segment
                if self.route_idx < len(self.route) - 1:

                    # move to next seg
                    self.route_idx += 1
                    next_seg_idx = self.route[self.route_idx]
                    next_seg = self.segments[next_seg_idx]

                    # coordinates start next seg
                    start_lat = next_seg["lat"][0]
                    start_lon = next_seg["lon"][0]

                    # compute distance
                    d = (self.robot_lat - start_lat)**2 + (self.robot_lon - start_lon)**2

                    if d < (self.speed * 2)**2:
                        # Switch to next seg
                        print(f"Switching segment: {self.active_idx} -> {next_seg_idx}")
                        self.active_idx = next_seg_idx
                        self.seg = self.segments[self.active_idx]
                        self.ptr = 0
                        self.robot_lat = start_lat
                        self.robot_lon = start_lon
                    else:
                        # Move towards next segment start
                        self.move_towards(start_lat, start_lon)

                else:
                    # ROUTE COMPLETED
                    print("🎉 Route completed.")

                    # FIX for Matplotlib crash
                    try:
                        if self._timer:
                            self._timer.remove_callback(self.anim._step)
                            self._timer.stop()
                    except Exception:
                        pass

                    self._done = True
                    return

        # Draw robot
        self.ax.plot(self.robot_lon, self.robot_lat, "ro", markersize=ROBOT_MARKER_SIZE)
        self.ax.text(self.robot_lon + 0.00002, self.robot_lat, "Robot")

        self.ax.set_xlabel("Longitude")
        self.ax.set_ylabel("Latitude")
        self.ax.set_aspect("equal")
        self.ax.grid(True)

    # ============================
    #      OFFSET WAYPOINTS
    # ============================
    def compute_offset_waypoints(self, waypoints, offset_distance=1.0, direction="right"):
        n = len(waypoints)
        offset_wp = []

        for i in range(n):
            if i < n - 1:
                lat1, lon1 = waypoints[i]
            else:
                lat1, lon1 = waypoints[i-1]

            if i == 0:
                lat_prev, lon_prev = waypoints[i]
                lat_next, lon_next = waypoints[i + 1]
            elif i == n - 1: 
                lat_prev, lon_prev = waypoints[i - 1]
                lat_next, lon_next = waypoints[i]
            else:
                lat_prev, lon_prev = waypoints[i - 1]
                lat_next, lon_next = waypoints[i + 1]

            prev_east, prev_north = self.latlon_to_local(lat1, lon1, lat_prev, lon_prev)
            curr_east, curr_north = self.latlon_to_local(lat1, lon1, lat1, lon1)
            next_east, next_north = self.latlon_to_local(lat1, lon1, lat_next, lon_next)

            vector_prev = np.array([curr_east - prev_east, curr_north - prev_north])
            vector_next = np.array([next_east - curr_east, next_north - curr_north])

            if np.linalg.norm(vector_prev) > 1e-6:
                vector_prev /= np.linalg.norm(vector_prev)
            if np.linalg.norm(vector_next) > 1e-6:
                vector_next /= np.linalg.norm(vector_next)

            vector_avg = vector_prev + vector_next
            if np.linalg.norm(vector_avg) < 1e-6:
                vector_avg = vector_next

            theta = math.atan2(vector_avg[1], vector_avg[0])

            if direction == "right":
                off_east =  offset_distance * math.sin(theta)
                off_north = -offset_distance * math.cos(theta)
            else:
                off_east = -offset_distance * math.sin(theta)
                off_north =  offset_distance * math.cos(theta)

            # if i > 0:
            #     prev_lat, prev_lon = offset_wp[-1]
            #     prev_wp_east, prev_wp_north = self.latlon_to_local(lat1, lon1, prev_lat, prev_lon)
            #     curr_wp_east, curr_wp_north = off_east, off_north

            #     dist = np.hypot(curr_wp_east - prev_wp_east, curr_wp_north - prev_wp_north)

            #     if dist < THRESH_PT_OVERLAP:
            #         avg_off_east = np.average([off_east, prev_wp_east])
            #         avg_off_north = np.average([off_north, prev_wp_north])

            #         avg_lat, avg_lon = self.local_to_latlon(lat1, lon1, avg_off_east, avg_off_north)
            #         offset_wp[-1] = (avg_lat, avg_lon)
            #         continue

            new_lat, new_lon = self.local_to_latlon(lat1, lon1, off_east, off_north)
            offset_wp.append((new_lat, new_lon))

        return np.array(offset_wp)
    
    # ============================
    #          SHOW PLOT
    # ============================
    def visualize_plot(self, waypoints, offset_r, offset_l):
        plt.figure(figsize=(10,7))
        plt.plot(waypoints[:,1], waypoints[:,0], '-o', label="Original")
        plt.plot(offset_r[:,1], offset_r[:,0], '-o', label="Right Offset 1m")
        plt.plot(offset_l[:,1], offset_l[:,0], '-o', label="Left Offset 1m")
        plt.axis("equal")
        plt.legend()
        plt.show()
    
    def visualize_graph(self, segments, graph):
        plt.figure(figsize=(10, 8))

        # ambil lokasi node (pakai titik awal segmen)
        xs = []
        ys = []
        for seg in segments:
            xs.append(seg["lon"][0])
            ys.append(seg["lat"][0])

        # gambar node
        plt.scatter(xs, ys, s=80, c='blue')

        # beri nomor node
        for i, (x, y) in enumerate(zip(xs, ys)):
            plt.text(x, y, f"{i}", fontsize=12, color='black')

        # gambar edge
        for i, nbrs in graph.items():
            x1, y1 = xs[i], ys[i]
            for j in nbrs:
                x2, y2 = xs[j], ys[j]
                plt.plot([x1, x2], [y1, y2], 'r-', linewidth=1)

        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.title("Segment Graph Visualization")
        plt.gca().set_aspect("equal", adjustable="box")
        plt.grid(True)
        plt.show()

class waypoint_router(Node):
    def __init__(self):
        super().__init__("waypoint_router")
        # ============================
        #          LOGGER SETUP
        # ============================
        logger.remove()
        logger.add(
            sys.stdout,
            colorize=True,
            format="<green>{time:HH:mm:ss.SSS}</green> | <level>{level:^6}</level> | <cyan>{function}</cyan>:<cyan>{line}</cyan> - {message}",
            enqueue=True,
        )
        logger.info("Waypoint Router Initialized")

        self.pkg_path = get_package_share_directory("world_model")

        self.WPS_FOLDER = os.path.join(self.pkg_path, "../../../..", "src/world_model/waypoints/test")
        self.DST_FILE = os.path.join(self.pkg_path, "../../../..", "src/world_model/waypoints/destinations/rc_robotik.csv")
        self.segments = self.load_all_segments(self.WPS_FOLDER)
        self.destinations = self.load_destinations_csv(self.DST_FILE)

        logger.info(f"Loaded {len(self.segments)} segments from {self.WPS_FOLDER}")

        # self.graph = self.build_graph(self.segments, threshold_meters=5.0)
        # logger.info(f"Graph built with {len(self.graph)} nodes")

        # Print graph connections
        # for node, neighbors in graph.items():
        #     logger.info(f"Segment {node + 1} connects to segments: {[n + 1 for n in neighbors]}")


        # ============================
        #      Global Variables
        # ============================
        self.gps_lat = 0
        self.gps_lon = 0
        self.dst_lat = 0
        self.dst_lon = 0
        self.dst_idx = 0
        self.got_pose = 0
        self.got_final_wps = 0
        self.final_wps = []
        self.car_orientation_rad = 0.0

        # ============================
        #       ROS PARAMETERS
        # ============================
        self.declare_parameter("gps_topic", "/fix")
        self.declare_parameter("timer_period", 1.0)
        self.declare_parameter("destinations_file", self.DST_FILE)
        self.declare_parameter("offset_degree", 0.0)

        # Ambil nilai parameter
        self.gps_topic: str = self.get_parameter("gps_topic").value
        self.period: float = self.get_parameter("timer_period").value
        self.DST_FILE: str = self.get_parameter("destinations_file").value
        self.offset_degree: float = self.get_parameter("offset_degree").value

        self.get_logger().info(
            f"Params:\n"
            f"  gps_topic = {self.gps_topic}\n"
            f"  timer_period = {self.period}\n"
            f"  destinations_file = {self.DST_FILE}\n"
        )

        # ============================
        #       ROS SUBSCRIPTION
        # ============================
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            1
        )
        self.gps_sub_filtered = self.create_subscription(
            NavSatFix,
            '/gps/filtered',
            self.gps_callback_filtered,
            1
        )
        self.odom_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/slam/localization_pose',
            self.odom_callback,
            1
        )

        # ============================
        #        ROS PUBLISHER
        # ============================
        self.path_pub = self.create_publisher(
            Path,
            '/waypoint_router/path',
            1
        )
        self.gps_pose_fix_pub = self.create_publisher(
            PoseStamped,
            '/waypoint_router/gps_pose_fix',
            1
        )
        self.gps_pose_filtered_pub = self.create_publisher(
            PoseStamped,
            '/waypoint_router/gps_pose_filtered',
            1
        )

        # ============================
        #        TF LISTENER
        # ============================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ============================
        #          ROS TIMER
        # ============================
        self.timer = self.create_timer(self.period, self.timer_callback)
        self.timer.cancel()
    

    # ============================
    #          ROS STUFF
    # ============================
    def gps_callback(self, msg):
        self.gps_lat = msg.latitude
        self.gps_lon = msg.longitude

        tf = self.get_map_to_base_transform()
        if tf is None:
            self.get_logger().warn("No TF map→base_link, skipping path publish")
            return

        q = tf.transform.rotation
        yaw = self.quaternion_to_yaw(q)

        cos_yaw = math.cos(-yaw + math.radians(self.offset_degree))
        sin_yaw = math.sin(-yaw + math.radians(self.offset_degree))

        e, n = self.latlon_to_local(-7.278155399999999, 112.7974098, self.gps_lat, self.gps_lon)

        e_rot = e * cos_yaw - n * sin_yaw
        n_rot = e * sin_yaw + n * cos_yaw

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "base_link"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = e_rot * 56.8
        pose_msg.pose.position.y = n_rot * 56.8
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.gps_pose_fix_pub.publish(pose_msg)

        if self.got_pose == 0:
            self.got_pose = 1
            logger.info(f"Initial GPS fix acquired: lat={self.gps_lat:.6f}, lon={self.gps_lon:.6f}")
            self.timer.reset()

        # logger.info(f"GPS Update: lat={self.gps_lat:.6f}, lon={self.gps_lon:.6f}")

    def gps_callback_filtered(self, msg):
        gps_lat = msg.latitude
        gps_lon = msg.longitude

        tf = self.get_map_to_base_transform()
        if tf is None:
            self.get_logger().warn("No TF map→base_link, skipping path publish")
            return

        q = tf.transform.rotation
        yaw = self.quaternion_to_yaw(q)

        cos_yaw = math.cos(-yaw + math.radians(self.offset_degree))
        sin_yaw = math.sin(-yaw + math.radians(self.offset_degree))

        e, n = self.latlon_to_local(-7.278155399999999, 112.7974098, gps_lat, gps_lon)

        e_rot = e * cos_yaw - n * sin_yaw
        n_rot = e * sin_yaw + n * cos_yaw

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "base_link"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = e_rot * 56.8
        pose_msg.pose.position.y = n_rot * 56.8
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.gps_pose_filtered_pub.publish(pose_msg)
       
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.car_orientation_rad = yaw

    def timer_callback(self):
        # logger.info(f"Timer triggered at GPS: lat={self.gps_lat:.6f}, lon={self.gps_lon:.6f}")

        # if self.dst_idx >= self.destinations[0].shape[0]:
        #     logger.info("All destinations have been processed.")
        #     self.timer.cancel()
        #     return
        # else:
        #     logger.info(f"Processing destination {self.dst_idx + 1} of {self.destinations[0].shape[0]}")
        #     self.dst_lat = self.destinations[0][self.dst_idx]
        #     self.dst_lon = self.destinations[1][self.dst_idx]

        #     if self.has_arrived(self.gps_lat, self.gps_lon, self.dst_lat, self.dst_lon, threshold_meters=5.0):
        #         logger.info(f"Arrived at destination {self.dst_idx + 1}: lat={self.dst_lat:.6f}, lon={self.dst_lon:.6f}")
        #         self.dst_idx += 1
        #         self.got_final_wps = 0
        #         self.final_wps = []
        #         return
            
        #     if self.got_final_wps == 0:
        #         current_segment, wp_idx, d = self.find_nearest_segment(self.segments, self.gps_lat, self.gps_lon)
        #         goal_segment, _, _ = self.find_nearest_segment(self.segments, self.dst_lat, self.dst_lon)
        #         START = current_segment
        #         GOAL = goal_segment
        #         route, total_cost = self.dijkstra(self.segments, self.graph, START, GOAL)
        #         logger.info(f"Route to destination {self.dst_idx + 1}: {route} with total cost {total_cost:.2f}")

        #         if route is None:
        #             logger.error(f"No route found to destination {self.dst_idx + 1}: lat={self.dst_lat:.6f}, lon={self.dst_lon:.6f}")
        #             return

        #         self.final_wps = self.generate_waypoints_from_route(self.segments, route, current_segment, wp_idx, self.dst_lat, self.dst_lon)
        #         logger.info(f"Generated final waypoints to destination {self.dst_idx + 1}: {len(self.final_wps)} points")
        #         self.got_final_wps = 1

        tf = self.get_map_to_base_transform()
        if tf is None:
            self.get_logger().warn("No TF map→base_link, skipping path publish")
            return
        
        q = tf.transform.rotation
        yaw = self.quaternion_to_yaw(q)

        cos_yaw = math.cos(-yaw + math.radians(self.offset_degree))
        sin_yaw = math.sin(-yaw + math.radians(self.offset_degree))

        self.dst_lat = self.destinations[0][self.dst_idx]
        self.dst_lon = self.destinations[1][self.dst_idx]

        current_segment, wp_idx, d = self.find_nearest_segment(self.segments, self.gps_lat, self.gps_lon)
        goal_segment, _, _ = self.find_nearest_segment(self.segments, self.dst_lat, self.dst_lon)

        # logger.info(f"current_segment={current_segment}, wp_idx={wp_idx}, goal_segment={goal_segment}")
        # self.get_logger().info(f"current_segment={current_segment}, wp_idx={wp_idx}, goal_segment={goal_segment}")

        START = current_segment
        GOAL = goal_segment

        # route, total_cost = self.dijkstra(self.segments, self.graph, START, GOAL)
        # self.final_wps = self.generate_waypoints_from_route(self.segments, route, current_segment, wp_idx, self.dst_lat, self.dst_lon)
        # crop_wps = self.crop_waypoints_by_distance(self.final_wps, self.gps_lat, self.gps_lon, max_distance=100.0)

        crop_wps = self.load_waypoints_csv(self.WPS_FOLDER + "/gps_log2.csv")

        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for lat, lon in crop_wps:
            e, n = self.latlon_to_local_robot(
                self.gps_lat, self.gps_lon,
                self.gps_lat, self.gps_lon,
                lat, lon
            )

            e_rot = e * cos_yaw - n * sin_yaw
            n_rot = e * sin_yaw + n * cos_yaw

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = e_rot * 56.8
            pose_stamped.pose.position.y = n_rot * 56.8
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
        
        # logger.info(f"Published {len(crop_wps)} waypoints (cropped)")
        self.path_pub.publish(path_msg)

    def get_map_to_base_transform(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            return tf
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None
        
    def quaternion_to_yaw(self, q):
        # q = geometry_msgs.msg.Quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ============================
    #         ROAD LOADER
    # ============================
    def load_all_segments(self, folder):
        files = glob.glob(os.path.join(folder, "*.csv"))
        files = sorted(files, key=self.natural_sort)
        segments = []

        for f in files:
            seg = self.load_segments_csv(f)
            if len(seg["lat"]) == 0:
                logger.warning(f"Empty segment skipped: {seg['name']}")
                continue

            if len(seg["lat"]) < 2:
                logger.warning(f"Segment too short skipped: {seg['name']}")
                continue

            segments.append(seg)
        return segments

    def load_segments_csv(self, path):
        id = []
        lat = []
        lon = []

        with open(path, "r") as f:
            reader = csv.reader(f)
            next(reader)
            
            for row in reader:
                if len(row) < 3:
                    continue
                try:
                    id.append(int(row[0]))
                    lat.append(float(row[1]))
                    lon.append(float(row[2]))
                except:
                    continue

        lat = np.array(lat)
        lon = np.array(lon)
        id = np.array(id)

        seg = {
            "lat": lat,
            "lon": lon,
            "id": id
        }

        return seg

    def load_waypoints_csv(self, path):
        wps = []  # akan berisi [lat, lon]

        with open(path, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # skip header: id,latitude,longitude

            for row in reader:
                # row[0] = id
                # row[1] = latitude
                # row[2] = longitude
                lat = float(row[1])
                lon = float(row[2])
                wps.append([lat, lon])

        return np.array(wps)   # shape: (N, 2)

    def load_destinations_csv(self, path):
        lat = []
        lon = []

        with open(path, 'r') as f:
            reader = csv.reader(f)
            next(reader)

            for row in reader:
                lat.append(float(row[0]))
                lon.append(float(row[1]))
        return np.array(lat), np.array(lon)
    
    def natural_sort(self,text):
        return [int(num) if num.isdigit() else num
                for num in re.split(r'(\d+)', text)]
    
    def generate_waypoints_from_route(self, segments, route, start_seg_idx, start_wp_idx, dst_lat=None, dst_lon=None):
        final_wps = []

        first_seg = segments[start_seg_idx]
        for k in range(start_wp_idx, len(first_seg["lat"])):
            final_wps.append((first_seg["lat"][k], first_seg["lon"][k]))

        for seg_idx in route[1:]:
            seg = segments[seg_idx]

            if(seg_idx == route[-1] and dst_lat is not None and dst_lon is not None):
                for k in range(len(seg["lat"])):
                    lat_k = seg["lat"][k]
                    lon_k = seg["lon"][k]
                    final_wps.append((lat_k, lon_k))

                    if self.has_arrived(lat_k, lon_k, dst_lat, dst_lon, threshold_meters=5.0):
                        return final_wps
                continue

            for k in range(len(seg["lat"])):
                final_wps.append((seg["lat"][k], seg["lon"][k]))

        return final_wps

    def crop_waypoints_by_distance(self, waypoints, lat_now, lon_now, max_distance=30.0):
        cropped = []
        total_dist = 0.0

        idx, _ = self.find_nearest_wp(waypoints, lat_now, lon_now)
        if idx >= len(waypoints):
            return cropped

        last_lat, last_lon = waypoints[idx]
        cropped.append((last_lat, last_lon))

        for k in range(idx + 1, len(waypoints)):
            lat, lon = waypoints[k]

            d = self.haversine(last_lat, last_lon, lat, lon)
            if total_dist + d > max_distance:
                break

            cropped.append((lat, lon))
            total_dist += d
            last_lat, last_lon = lat, lon

        return cropped

    def find_nearest_segment(self, segments, lat_now, lon_now):
        best_seg = None
        best_wp = None
        best_d = float("inf")

        for i, seg in enumerate(segments):
            for wp_idx in range(len(seg["lat"])):
                d = self.haversine(lat_now, lon_now, seg["lat"][wp_idx], seg["lon"][wp_idx])
                if d < best_d:
                    best_d = d
                    best_seg = i
                    best_wp = wp_idx

        return best_seg, best_wp, best_d
    
    def find_nearest_wp(self, waypoints, lat_now, lon_now):
        best_idx = 0
        best_d = float("inf")

        for i, (lat, lon) in enumerate(waypoints):
            d = self.haversine(lat_now, lon_now, lat, lon)
            if d < best_d:
                best_d = d
                best_idx = i

        return best_idx, best_d

    def has_arrived(self, lat1, lon1, lat2, lon2, threshold_meters=5.0):
        d = self.haversine(lat1, lon1, lat2, lon2)
        return d <= threshold_meters

    # ============================
    #       LAT/LON STUFF
    # ============================
    def latlon_to_local(self, lat0, lon0, lat, lon):
        """Project 3D Earth into 2D Map"""
        dLat = math.radians(lat - lat0)
        dLon = math.radians(lon - lon0)
        lat0_rad = math.radians(lat0)

        dNorth = dLat * METER_PER_DEGREE
        dEast  = dLon * (METER_PER_DEGREE * math.cos(lat0_rad))
        return dEast, dNorth
    
    def latlon_to_local_robot(self, lat_map0, lon_map0,
                          lat_robot, lon_robot,
                          lat_wp, lon_wp):
        wp_e, wp_n = self.latlon_to_local(lat_map0, lon_map0, lat_wp, lon_wp)
        robot_e, robot_n = self.latlon_to_local(lat_map0, lon_map0, lat_robot, lon_robot)
        rel_e = wp_e - robot_e
        rel_n = wp_n - robot_n

        return rel_e, rel_n

    def local_to_latlon(self, lat0, lon0, east, north):
        """Deproject 2D Map into 3D Earth"""
        dLat = north / METER_PER_DEGREE
        dLon = east  / (METER_PER_DEGREE * math.cos(math.radians(lat0)))
        return lat0 + dLat, lon0 + dLon
    
    def haversine(self, lat1, lon1, lat2, lon2):
        """Phytagoras-like for 3D Earth"""
        R = 6371000
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
        return 2 * R * math.asin(math.sqrt(a))

    # ============================
    #        PATH PLANNING
    # ============================
    def build_graph(self, segments, threshold_meters=15.0):
        g = {}
        for i, A in enumerate(segments):
            g[i] = []
            for j, B in enumerate(segments):
                if i == j:
                    continue
                if self.is_connected(A, B, threshold_meters):
                    g[i].append(j)

        incoming = {dst for nbrs in g.values() for dst in nbrs}
        disconnected = [i for i, nbrs in g.items() if len(nbrs) == 0 and i not in incoming]

        if len(disconnected) > 0:
            logger.error(f"Found {len(disconnected)} completely disconnected node(s): {disconnected}")
            raise RuntimeError(f"Completely disconnected node(s) detected: {disconnected}")

        return g
    
    def is_connected(self, segA, segB, threshold_meters=15.0):
        latA = segA["lat"][-1]
        lonA = segA["lon"][-1]
        latB = segB["lat"][0]
        lonB = segB["lon"][0]
        d = self.haversine(latA, lonA, latB, lonB)

        return d <= threshold_meters
        
    def segment_cost(self, segA, segB, threshold_meters=15.0):
        # jarak dari A.end ke B.start
        d = self.haversine(
            segA["lat"][-1], segA["lon"][-1],
            segB["lat"][0],  segB["lon"][0]
        )

        ret = d
        if d > threshold_meters:
            ret += 99999.0
        else:
            ret = segB["id"].shape[0]

        return ret
    
    def dijkstra(self,segments, graph, start, goal, threshold_meters=15.0):
        # priority queue: (cost, node, path)
        pq = [(0, start, [start])]
        visited = set()

        while pq:
            cost, node, path = heapq.heappop(pq)
            if node == goal:
                return path, cost

            if node in visited:
                continue
            visited.add(node)

            for nb in graph[node]:
                edge_cost = self.segment_cost(segments[node], segments[nb])
                heapq.heappush(pq, (cost + edge_cost, nb, path+[nb]))

        return None, float("inf")

def main(args=None):
    rclpy.init(args=args)
    node = waypoint_router()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)