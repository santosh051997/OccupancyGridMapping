#!/usr/bin/env python3

import rclpy
import numpy as np
import matplotlib.pyplot as plt
from time import perf_counter
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

from grid_map import *
from message_handler import * 
from utils import *

def main():
      try:
                  # BAG_FILE_PATH = '/home/santosh/ros2mapping_ws/src/grid_mapping/bagfiles'
                  MAPS_PATH = '/home/santosh/ros2mapping_ws/src/grid_mapping/maps'
                  P_prior = 0.5	# Prior occupancy probability
                  P_occ = 0.9	# Probability that cell is occupied with total confidence
                  P_free = 0.3	# Probability that cell is free with total confidence 
                  RESOLUTION = 0.04 # Grid resolution in [m]
                  MAP_NAME = BAG_FILE_NAME = 'rosbag2_2024_04_24-14_08_46' # map and bagfile name without extension

                  N_odom = 0
                  N_scan = 0
                  N_paired = 0
                  N_unpaired = 0

                  msgHanlder = MsgHandler()

                  map_x_lim = [-10, 10]
                  map_y_lim = [-6, 6]
                  dir_pointer_len = 0.25

                  # Create grid map 
                  gridMap = GridMap(X_lim = map_x_lim, 
                                    Y_lim = map_y_lim, 
                                    resolution = RESOLUTION, 
                                    p = P_prior)
                  
                  # Init figure
                  plt.style.use('seaborn-ticks')
                  fig = plt.figure(1)
                  ax = fig.add_subplot(1,1,1)

                  # Init time
                  t_start = perf_counter()
                  sim_time = 0
                  step = 0

                  print('\n*** Displaying rosbag files ***')

                  typestore = get_typestore(Stores.LATEST)
                  with Reader('/home/santosh/ros2mapping_ws/src/grid_mapping/bagfiles/rosbag2_2024_04_24-14_08_46') as reader:
                        for connection, timestamp, rawdata in reader.messages():
                              if connection.topic == '/scan':
                                    N_scan += 1
                              elif connection.topic == '/odom':
                                    N_odom += 1
                              
                              msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                              with open ('writeme1.txt', 'w') as file:
                                     file.write(str(msg))  

                        
                              msgHanlder.accept_message(connection.topic, msg)

                              # if message pair (Odometry+Scan) is not complete continue reading
                              if msgHanlder.pair_check() == False:
                                          N_unpaired += 1
                                          continue
                              else:
                                          N_paired +=1
                                                
                              msgOdom, msgScan = msgHanlder.process_paired_messages()

                              # Odometry message processing
                              x_odom, y_odom = get_odom_position(msgOdom)   # x,y in [m]
                              theta_odom = get_odom_orientation(msgOdom)    # theta in [radians]

                              # LidarScan message processing
                              distances, angles, information = lidar_scan(msgScan) # distances in [m], angles in [radians], information [0-1]

                              # Lidar measurements in X-Y plane 
                              distances_x, distances_y = lidar_scan_xy(distances, angles, x_odom, y_odom, theta_odom)
                              filtered_distances_x = []
                              filtered_distances_y = []
                        
                              # x1 and y1 for Bresenham's algorithm
                              x1, y1 = gridMap.discretize(x_odom, y_odom)

                              # for BGR image of the grid map
                              X2 = []
                              Y2 = [] 
                        
                              for (dist_x, dist_y, dist) in zip(distances_x, distances_y, distances):

                                          # x2 and y2 for Bresenham's algorithm
                                          x2, y2 = gridMap.discretize(dist_x, dist_y)

                                          # draw a discrete line of free pixels, [robot position -> laser hit spot)
                                          for (x_bres, y_bres) in bresenham(gridMap, x1, y1, x2, y2):
                                                gridMap.update(x = x_bres, y = y_bres, p = P_free)

                                          # mark laser hit spot as ocuppied (if exists)
                                          if dist < msgScan.range_max:
                                                gridMap.update(x = x2, y = y2, p = P_occ)

                                                # filtered distances in X-Y plane for Ploting
                                                filtered_distances_x.append(dist_x)
                                                filtered_distances_y.append(dist_y)

                                          # for BGR image of the grid map
                                          X2.append(x2)
                                          Y2.append(y2)

                              # converting grip map to BGR image
                              bgr_image = gridMap.to_BGR_image()

                              # marking robot position with blue pixel value
                              set_pixel_color(bgr_image, x1, y1, 'BLUE')
                                    
                              # marking neighbouring pixels with blue pixel value 
                              for (x, y) in gridMap.find_neighbours(x1, y1):
                                    set_pixel_color(bgr_image, x, y, 'BLUE')

                              # marking laser hit spots with green value
                              for (x, y) in zip(X2,Y2):
                                    set_pixel_color(bgr_image, x, y, 'GREEN')
                                    
                              resized_image = cv2.resize(src = bgr_image, 
                                                      dsize = (500, 500), 
                                                      interpolation = cv2.INTER_AREA)
                              
                              rotated_image = cv2.rotate(src = resized_image, 
                                                      rotateCode = cv2.ROTATE_90_COUNTERCLOCKWISE)
                              
                              cv2.imshow("Grid map", rotated_image)
                              cv2.waitKey(1)

                              ax.clear()

                              # Plot Lidar measurements
                              ax.plot(filtered_distances_x, filtered_distances_y, 'b.', markersize = 1.2, label = 'lidar')

                              # Plot Robot position
                              dir_robot_x = np.array([x_odom, x_odom + dir_pointer_len * np.cos(theta_odom)])
                              dir_robot_y = np.array([y_odom, y_odom + dir_pointer_len * np.sin(theta_odom)])

                              ax.plot(x_odom, y_odom, 'r.', markersize = 8, label = 'robot')
                              ax.plot(dir_robot_x, dir_robot_y, color = 'red', lw = 1.5)

                              plt.xlabel('x[m]')
                              plt.ylabel('y[m]')
                              plt.title(BAG_FILE_NAME + '.bag')
                              plt.xlim(map_x_lim)
                              plt.ylim(map_y_lim)
                              plt.draw()
                              plt.pause(0.0001)

                              # Calculate step time in [s]
                              t_step = perf_counter()
                              step_time = t_step - t_start
                              sim_time += step_time
                              t_start = t_step
                              step += 1 

                              print('Step %d ==> %d [ms]' % (step, step_time * 1000))

                  # Terminal outputs
                  print('\nScan messages: %d' % N_scan)
                  print('Odom messages: %d' % N_odom)
                  print('Paired messages: %d' % N_paired)
                  print('Unpaired messages: %d' % N_unpaired)
                  
                  print('\nSimulation time: %.2f [s]' % sim_time)
                  print('Average step time: %d [ms]' % (sim_time * 1000 / step))
                  print('Frames per second: %.1f' % (step / sim_time))

                  # Saving Grid Map
                  resized_image = cv2.resize(src = gridMap.to_BGR_image(), 
                                                      dsize = (500, 500), 
                                                      interpolation = cv2.INTER_AREA)

                  rotated_image = cv2.rotate(src = resized_image, 
                                    rotateCode = cv2.ROTATE_90_COUNTERCLOCKWISE)

                  flag_1 = cv2.imwrite(img = rotated_image * 255.0, 
                              filename = MAPS_PATH + '/' + MAP_NAME + '_GRID_MAP.png')

                  # Calculating Maximum likelihood estimate of the map
                  gridMap.calc_MLE()

                  # Saving MLE of the Grid Map
                  resized_image_MLE = cv2.resize(src = gridMap.to_BGR_image(), 
                                          dsize = (500, 500), 
                                          interpolation = cv2.INTER_AREA)

                  rotated_image_MLE = cv2.rotate(src = resized_image_MLE, 
                                          rotateCode = cv2.ROTATE_90_COUNTERCLOCKWISE)

                  flag_2 = cv2.imwrite(img = rotated_image_MLE * 255.0, 
                              filename = MAPS_PATH + '/' + MAP_NAME + '_GRID_MAP_MLE.png')

                  if flag_1 and flag_2:
                        print('\nGrid map successfully saved!\n')

                  if cv2.waitKey(0) == 27:
                        cv2.destroyAllWindows()
     
      except rclpy.exceptions.ROSInterruptException:
             print('\r\nSIMULATION TERMINATED!')
             pass
                
if __name__ == '__main__':
    main()