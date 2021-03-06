from PIL import Image
import numpy as np
import colorsys

import os

np.set_printoptions(precision=2, suppress=True, threshold=np.nan)


bit_depth = {'1': 1, 'L': 8, 'LA': 16, 'P': 8, 'RGB': 8, 'RGBA': 8, 'CMYK': 8, 'YCbCr': 8, 'I': 32, 'F': 32, 'HSV': 8}
channel_depth = {'1': 1, 'L': 1, 'LA': 2, 'P': 1, 'RGB': 3, 'RGBA': 4, 'CMYK': 4, 'YCbCr': 3, 'I': 1, 'F': 1, 'HSV': 3}


class Raster(object):

    def __init__(self, colors, shape, mode, mask=None, name=None):

        self._shape = np.array(shape)
        self._colors = np.array(colors)
        self._mask = np.array(mask)
        self._mode = mode
        self._name = name

    @property
    def name(self):
        return self._name

    @property
    def shape(self):
        return self._shape

    @property
    def colors(self):
        return self._colors

    @colors.setter
    def colors(self, values):
        values = np.array(values).astype(np.float64)

        if len(values) != np.product(self._shape):
            raise ValueError("Length of colors does not match shape")

        self._colors = np.array(values)

    @property
    def mask(self):
        return self._mask

    @mask.setter
    def mask(self, mask):
        if mask is None:
            mask = np.ones(np.product(self._shape))

        mask = np.array(mask).astype(np.float64)
        if len(mask) != np.product(self._shape):
            raise ValueError("Mask must have same number of elements as colors")

        if np.max(mask) > 1 or np.min(mask) < 0:
            raise ValueError("Mask elements must be between zero and one")
        self._mask = np.array(mask).astype(np.float64)

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, mode):
        if mode not in channel_depth.keys():
            raise ValueError("Mode is not recognized")
        if channel_depth[mode] != np.array(self._colors).shape[1]:
            raise ValueError("Color mode incompatible with number of channels given")
        self._mode = mode

    @classmethod
    def from_image(cls, image, mode=None, name=None):

        if mode is not None:
            image = image.convert(mode)
        else:
            mode = image.mode

        # Convert to flat numpy array
        channels = channel_depth[mode]
        width, height = image.size
        pixels = np.asarray(image.getdata()).reshape((width * height, channels))

        # Normalize data
        bits = bit_depth[image.mode]
        pixels = pixels.astype(np.float64) / (2**bits - 1)

        # Separate mask
        colors = pixels[:, :channels-1]
        mask = pixels[:, channels-1]

        return cls(colors, image.size, image.mode.replace('A', ''), mask, name=name)

    @classmethod
    def from_path(cls, path, mode=None):
        name = os.path.split(path)[1]
        return cls.from_image(Image.open(path), mode, name=name)

    # Combines alpha channel with color
    def with_alpha(self):
        """Returns data with alpha"""
        mask = np.reshape(self._mask, (len(self._mask), 1))
        return np.append(self._colors, mask, axis=1)

    def get_image(self):
        self.to_rgb()
        pixels = self.get_tiered()
        return Image.fromarray(np.ceil(pixels * 255).astype(np.uint8), mode='RGBA')

    def get_opaque(self):
        return np.array([np.append(color, alpha) for color, alpha in zip(self._colors, self._mask) if alpha > 0.])

    def get_tiered(self):
        return np.reshape(self.with_alpha(), (self._shape[1], self._shape[0], 4))

    @classmethod
    def from_array(cls, array, mode):

        channels = channel_depth[mode]

        width, height = array.shape
        array = np.reshape(array, (np.product(array.shape), 4))
        # Normalize data
        bits = bit_depth[mode]
        array = array.astype(np.float64) / (2**bits - 1)
        colors = array[:, :channels-1]
        mask = array[:, channels-1]
        return cls(colors, (width, height), mode.replace('A', ''), mask)

    def channel(self, identifier, opaque=False):
        if opaque:
            colors = self.get_opaque()
        else:
            colors = self._colors

        # REMINDER: Only works for HSV/RGB switching
        if identifier not in self._mode:
            if 'RGB' in self._mode:
                self.to_hsv()
            else:
                self.to_rgb()

        # Common error point- Palettized images are not cast into HSV, so the Value channel cannot be found.
        # Fix by setting the optional mode argument upon image load to RGB

        # print(self.mode)
        column = self._mode.index(identifier)
        return colors[:, column]

    def to_hsv(self):
        if 'RGB' in self._mode:
            for index, (r, g, b) in enumerate(self._colors):
                h, s, v = colorsys.rgb_to_hsv(r, g, b)
                self._colors[index] = [h, s, v]
                self._mode = 'HSV'

    def to_rgb(self):
        if 'HSV' in self._mode:
            for index, (h, s, v) in enumerate(self._colors):
                r, g, b = colorsys.hsv_to_rgb(h, s, v)
                self._colors[index] = [r, g, b]
                self._mode = 'RGB'

    def save(self, path):
        self.get_image().save(path)

    # Optional fragment-based manipulation
    def filter(self, func, *args):
        for index, pixel in enumerate(self._colors):
            self._colors[index] = func(pixel, *args)