# Simulated environment that the interface reads from
from Environment.Raster import Raster
import pygame
from pygame.locals import *

# Each pixel is an inch?

def load_environment(path):
    image = Raster.from_path(path)

    return image.get_tiered()

