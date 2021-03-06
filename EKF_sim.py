import pygame, sys
from EKF_car import CarSim
from EKF_filter import Filter
import numpy as np

# General Setup
pygame.init()
clock = pygame.time.Clock()

WIDTH, HEIGHT = 800, 600
WINDOW_SIZE = (WIDTH, HEIGHT)
dis = pygame.display.set_mode(WINDOW_SIZE)
screen = pygame.Surface((400, 300)) # 400, 300
pygame.display.set_caption('Kalman Filter Simulator')
SCREEN_MULTI = 3

running = True

car_sim = CarSim(100, 147, 400, 300)
filter = Filter(100, 147)
car_x, car_y = 100, 147

while running:
    # Events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_a:
                car_sim.keys['left'] = True
            if event.key == pygame.K_d:
                car_sim.keys['right'] = True
            if event.key == pygame.K_w:
                car_sim.keys['up'] = True
            if event.key == pygame.K_s:
                car_sim.keys['down'] = True
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_a:
                car_sim.keys['left'] = False
            if event.key == pygame.K_d:
                car_sim.keys['right'] = False
            if event.key == pygame.K_w:
                car_sim.keys['up'] = False
            if event.key == pygame.K_s:
                car_sim.keys['down'] = False
        
    screen.fill((80, 80, 200))
      
    # Update Car and publish Odom Results    
    car_sim.update()
    #Draws real car pose - red
    car_sim.draw(screen, car_sim.rect.x, car_sim.rect.y)
    odom_values = car_sim.publish_odom()
    lidar_values = [0, 0]
    lidar_values[0], lidar_values[1] = car_sim.publish_lidar()
    #print(f'odom values: {odom_values}, lidar front: {lidar_values}')

    pred_x, pred_y = filter.calc_pose(odom_values, lidar_values)
    lidar_x, lidar_y = lidar_values
    v_x, v_y = odom_values
    car_x += v_x
    car_y += v_y
    car_x, car_y = int(car_x), int(car_y)

    
    # Draws position using odom data - yellow/green
    pygame.draw.rect(screen, (200, 255, 80), pygame.Rect(car_x, car_y, car_sim.CAR_DIM[0], car_sim.CAR_DIM[1]))
    # Draws position using lidar data - blue
    pygame.draw.rect(screen, (80, 255, 200), pygame.Rect(400-lidar_x, 300-lidar_y, car_sim.CAR_DIM[0], car_sim.CAR_DIM[1]))
    # Draws position using kalman filter - Green
    pygame.draw.rect(screen, (80, 255, 80), pygame.Rect(pred_x, pred_y, car_sim.CAR_DIM[0], car_sim.CAR_DIM[1]))

    

    surf = pygame.transform.scale(screen, WINDOW_SIZE)
    dis.blit(surf, (0, 0))
    pygame.display.flip()
    clock.tick(60)