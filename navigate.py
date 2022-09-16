import time
import numpy as np
import picar_4wd as fc
import matplotlib.pyplot as plt
from picar_4wd.servo import Servo
from picar_4wd.pwm import PWM



theta_step = 6
grid_size = (100, 100)
servo_min_angle = -90
servo_max_angle = 90
car_start_cordinates = (59, 0)

def set_grid(grid,x,y):
    x = grid_size[0] - 1 if x >= grid_size[0] else x
    y = grid_size[1] - 1 if y >= grid_size[1] else y
    x,y = np.uint8(abs(x)), np.uint8(abs(y))
    grid[x,y] = 1
    return grid

# def adjust_coordinates_boundary(x,y):
#     print("I am called ...")
#     x = grid_size[0] - 1 if x >= grid_size[0] else x
#     y = grid_size[1] - 1 if y >= grid_size[1] else y
#     x,y = np.uint8(abs(x)), np.uint8(abs(y))
#     return x,y

def get_mapping():
    plannar_grid = np.zeros(shape= grid_size, dtype= np.uint8)
    ser = Servo( PWM("P0"))
    ser.set_angle(0)
    time.sleep(0.04)
    x0,y0 = 0,0

    for theta1 in range(servo_min_angle, servo_max_angle, theta_step):
        theta =  np.radians(theta1)
        dist = fc.get_distance_at(theta1)
        
        if dist <= 35 and dist != -2:
            x, y = car_start_cordinates[0] + np.uint8(dist * np.sin(theta)), car_start_cordinates[1] + np.uint8(dist * np.cos(theta))
            
            if x == x0:
                x1 = x0
                for _dy in range(0, abs(y-y0)):                                                                                                                                                                                                                                                                                                 
                    y1 = y0+_dy if y > y0 else y0-_dy
                    plannar_grid = set_grid(plannar_grid,x1,y1)
            else:
                m = slope = (y-y0)/(x-x0)
                interpolated_range = range(0, abs(x-x0) )
                
                for _dx in interpolated_range:
                    dx = _dx if x > x0 else -_dx
                    x1 = x0 + dx
                    y1 = y0 + m*dx
                    plannar_grid = set_grid(plannar_grid,x1,y1)

            plannar_grid = set_grid(plannar_grid,x,y)
            x0,y0 = x,y
    
    return plannar_grid



def main():
    total_dist = 30
    forward_moves  = 15      
    total_moves = 0
    TK_RIGHT_FLG = 1

    #np.set_printoptions(linewidth=np.inf)
    while total_moves < total_dist:
        floor_grid = get_mapping()
        move_speed = fc.Speed(10)
        move_speed.start()       

        # print(np.all(floor_grid[55:80, 0:10] == 0))
        #print(floor_grid[55:80, 0:10] )
        #print(floor_grid)
        # plt.imshow(floor_grid, origin='lower')
        # plt.show()
        
        if np.all(floor_grid == 0):
            if total_moves >= forward_moves and TK_RIGHT_FLG == 1:
                print("I am in Right")
                fc.turn_right(5)
                TK_RIGHT_FLG = 0
            else:
                fc.forward(10)
                total_moves += 6
                print(total_moves)
                move_speed.deinit()
        else:
            fc.stop()
            print(floor_grid)
            plt.imshow(floor_grid, origin='lower')
            plt.show()
    
    fc.stop()                             


if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
