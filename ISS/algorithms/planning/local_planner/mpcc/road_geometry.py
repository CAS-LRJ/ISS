import math
import numpy as np
import matplotlib.pyplot as plt
from math import pi

class CenterLinePoint:
    def __init__(self, px, py, heading):
        self.px = px
        self.py = py
        self.heading = heading
        
class CenterLine:
    def __init__(self, line_config, start_pose, resolution=0.1, lane_width=4):
        self.config = line_config
        self.start_pose = start_pose
        self.resolution = resolution
        self.lane_width = lane_width
        self.center_line_points = []
        
        self._construct_center_line()
        
    def _construct_center_line(self):
        
        px = self.start_pose[0]
        py = self.start_pose[1]
        heading = self.start_pose[2]
        self.center_line_points.append(CenterLinePoint(px, py, heading))
        
        for line_segment in self.config:
            segment_type = line_segment[0]
            # Arc
            if segment_type == 'arc':
                [radius, angle] = line_segment[1:3]
                arc_direction = np.sign(angle)
                start_angle = heading - arc_direction * pi/2 
                end_angle = start_angle + angle
                arc_length = math.fabs(angle) * radius

                center_heading = heading + arc_direction * pi/2
                arc_center_x = px + radius * math.cos(center_heading)
                arc_center_y = py + radius * math.sin(center_heading)
                
                # bug!
                # s_tmp = self.resolution
                # while(s_tmp <= arc_length):
                #     angle_tmp = start_angle + arc_direction * s_tmp / radius
                #     px = arc_center_x + radius * math.cos(angle_tmp)
                #     py = arc_center_y + radius * math.sin(angle_tmp)
                #     heading += arc_direction * self.resolution / radius                                        
                #     self.center_line_points.append(CenterLinePoint(px, py, heading))
                #     s_tmp += self.resolution
                
                num_points = int(np.floor(arc_length / self.resolution))+1
                angles = np.linspace(start_angle, end_angle, num_points)
                for i in range(1, num_points):
                    px = arc_center_x + radius * math.cos(angles[i])
                    py = arc_center_y + radius * math.sin(angles[i])
                    heading += angle / (num_points-1)
                    self.center_line_points.append(CenterLinePoint(px, py, heading))
                    
            # Straight line
            elif segment_type == 'line':
                length = line_segment[1]

                # bug
                # s_tmp = self.resolution
                # # s_tmp = 0.
                # while(s_tmp <= length):
                #     px += self.resolution * math.cos(heading)
                #     py += self.resolution * math.sin(heading)
                #     self.center_line_points.append(CenterLinePoint(px, py, heading))
                #     s_tmp += self.resolution
                
        
                num_points = int(np.floor(length / self.resolution))+1
                px_0 = px
                py_0 = py
                lengths = np.linspace(0., length, num_points)
                for i in range(1, num_points):
                    px = px_0 + lengths[i] * math.cos(heading)
                    py = py_0 + lengths[i] * math.sin(heading)
                    self.center_line_points.append(CenterLinePoint(px, py, heading))
                
            else:
                raise ValueError("Unknown segment type")
    
    def get_2Dpoints(self):
        num_points = len(self.center_line_points)
        p_xy = np.zeros((2, num_points))
        for i in range(num_points):
            p_xy[0, i] = self.center_line_points[i].px
            p_xy[1, i] = self.center_line_points[i].py
        return p_xy   
 
    def plot_center_line(self, show_boundaries=True, line_color='b'):
        num_points = len(self.center_line_points)
        px = np.zeros(num_points)
        py = np.zeros(num_points)
        
        px_left = np.zeros(num_points)
        py_left = np.zeros(num_points)
        px_right = np.zeros(num_points)
        py_right = np.zeros(num_points)
        
        for i in range(num_points):
            px[i] = self.center_line_points[i].px
            py[i] = self.center_line_points[i].py
            heading = self.center_line_points[i].heading
            
            px_left[i] = px[i] - self.lane_width/2 * math.sin(heading)
            py_left[i] = py[i] + self.lane_width/2 * math.cos(heading)
            
            px_right[i] = px[i] + self.lane_width/2 * math.sin(heading)
            py_right[i] = py[i] - self.lane_width/2 * math.cos(heading)
            
        plt.plot(px, py, line_color+'--')
        # plt.plot(px, py, '.')
        if show_boundaries:
            plt.plot(px_left, py_left, '-k')
            plt.plot(px_right, py_right, '-k')
        plt.axis('equal')
            
def main():
    # Line + Arc + Line + Arc + ...
    line_config = [
        ['line', 30],
        ['arc', 10, -pi/2],
        ['line', 10],
        ['arc', 5, pi],
        ['line', 36],
        ['arc', 12, -pi],
        ['line', 50]
    ]
    start_pose = [0., 0., 0.]
    center_line = CenterLine(line_config, start_pose)
    center_line.plot_center_line()
    plt.show()
 
if __name__ == '__main__':
    main()
