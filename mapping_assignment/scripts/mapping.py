#!/usr/bin/env python3

"""
    # {student Ernestas}
    # {student 19960408-T399}
    # {student ernestas@kth.se}
"""

# Python standard library
from math import cos, sin, atan2, fabs, radians, isinf, ceil

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate
from grid_map import GridMap

class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.

        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        l_min_angle = scan.angle_min        # start angle of the scan [rad]
        l_max_angle = scan.angle_max        # end angle of the scan [rad]
        l_inc_angle = scan.angle_increment  # angular distance between measurements [rad]
        l_min_range = scan.range_min        # minimum range value [m]
        l_max_range = scan.range_max        # maximum range value [m]
        l_ranges = scan.ranges

        # geometry_msgs/Pose
        r_pose = pose.pose
        # geometry_msgs/Quaternion
        r_orientation = pose.pose.orientation
        # geometry_msgs/Point
        r_position = pose.pose.position

        # Current yaw of the robot
        robot_yaw = self.get_yaw(r_orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()

        o01 = np.matrix([[r_position.x], 
                         [r_position.y]])
        o0 = np.matrix([[origin.position.x], 
                        [origin.position.y]])
        r01 = np.matrix([[np.cos(robot_yaw), -np.sin(robot_yaw)],
                         [np.sin(robot_yaw),  np.cos(robot_yaw)]])
        angle = l_min_angle
        rx = int( (r_position.x - origin.position.x) / resolution) 
        ry = int( (r_position.y - origin.position.y) / resolution)
        minx = np.PINF
        miny = np.PINF
        maxx = np.NINF
        maxy = np.NINF
        amap = np.full((grid_map.get_width(), grid_map.get_height()), self.unknown_space)
        for rng in l_ranges:
            if rng <= l_min_range or rng >= l_max_range:
                angle += l_inc_angle
                continue
            p0 = np.matrix([[np.cos(angle)*rng], 
                            [np.sin(angle)*rng]])
            p1 = r01.dot(p0) + o01 - o0
            x = int(p1.item(0) / resolution)
            y = int(p1.item(1) / resolution)
            laser_beams = self.raytrace((rx, ry), (x, y))
            for laser_beam in laser_beams:
                amap[laser_beam[0]][laser_beam[1]] = self.free_space
                self.add_to_map(grid_map, laser_beam[0], laser_beam[1], self.free_space)
                minx = min(laser_beam[0],minx)
                maxx = max(laser_beam[0],maxx)
                miny = min(laser_beam[1],miny)
                maxy = max(laser_beam[1],maxy)
            angle += l_inc_angle
            minx = min(x,minx)
            maxx = max(x,maxx)
            miny = min(y,miny)
            maxy = max(y,maxy)

        angle = l_min_angle
        for rng in l_ranges:
            if rng <= l_min_range or rng >= l_max_range:
                angle += l_inc_angle
                continue
            p0 = np.matrix([[np.cos(angle)*rng], 
                            [np.sin(angle)*rng]])
            p1 = r01.dot(p0) + o01 - o0
            x = int(p1.item(0) / resolution)
            y = int(p1.item(1) / resolution)
            amap[x][y] = self.occupied_space
            self.add_to_map(grid_map, x, y, self.occupied_space)
            angle += l_inc_angle

        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = minx
        # The minimum y index in 'grid_map' that has been updated
        update.y = miny
        # Maximum x index - minimum x index + 1
        update.width = maxx - minx + 1
        # Maximum y index - minimum y index + 1
        update.height = maxy - miny + 1
        try:
            data = []
            for i in range(update.width):
                for j in range(update.height):
                    data.append(amap[minx+i][miny+j])
        except:
            pass
        # The map data inside the rectangle, in row-major order.
        update.data = data

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.

        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """

        amap = np.array(grid_map.to_message().data).reshape((grid_map.get_width(),grid_map.get_height())).T
        motion = [[1, 0],
                [0, 1],
                [-1, 0],
                [0, -1],
                [-1, -1],
                [-1, 1],
                [1, -1],
                [1, 1]]
        for index, val in np.ndenumerate(amap):
            if val != self.occupied_space:
                continue
            ix, iy = index
            space = [index]
            closed = []
            while space:
                current = space.pop()
                for m in motion:
                    x,y = current
                    x += m[0]
                    y += m[1]
                    if not self.is_in_bounds(grid_map, x, y):
                        continue
                    if (x,y) in closed:
                        continue
                    if np.sqrt((x-ix)**2 + (y-iy)**2) > self.radius:
                        continue
                    if amap[x][y] != self.occupied_space:
                        self.add_to_map(grid_map, x, y, self.c_space)
                    if (x,y) not in space:
                        space.append((x,y))
                    closed.append((x,y))

        # Return the inflated map
        return grid_map