import sys
import pygame
from pygame.locals import *
import random

class graph:

    def __init__(self, data_num, scales, size):
        self.data_num = data_num
        self.scales = scales

        pygame.init()

        self.FPS = 60
        self.FramePerSec = pygame.time.Clock()

        self.BLUE = (0,0,255)
        self.RED = (255,0,0)
        self.GREEN = (0,255,0)
        self.BLACK = (0,0,0)
        self.WHITE = (255,255,255)
        
        self.colors = []
        for _ in range(self.data_num):
            r = random.randint(0, 255)
            g = random.randint(0, 255)
            b = random.randint(0, 255)
            self.colors.append([r, g, b])

        self.size = size
        self.screen = pygame.display.set_mode(self.size)
        self.screen.fill(self.WHITE)
        pygame.display.set_caption("graph") 
    
    def plot_dot(self, x, y, color, scale, circle_r = 1): #x-axis on left end, y-axis on middle
        pygame.draw.circle(self.screen, color, [x, int(self.size[1] / 2 - (y * scale))], circle_r)
    
    def run(self, *data_queues):
        x = 0
        while True:
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()
            
            while not data_queues[0].empty():
                for i in range(self.data_num):
                    self.plot_dot(x, data_queues[i].get(), self.colors[i], self.scales[i])
                x += 1
            if x > self.size[0]:
                self.screen.fill(self.WHITE)
                x = 0

            self.FramePerSec.tick(self.FPS)
            pygame.display.update()