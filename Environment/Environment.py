# Simulated environment that the interface reads from

from Environment.Raster import Raster


def load_environment(path):
    image = Raster.from_path(path)

    image.colors