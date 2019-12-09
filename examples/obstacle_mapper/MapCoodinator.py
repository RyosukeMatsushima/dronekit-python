import urllib.request
from PIL import Image
import os
import math


class MapCoodinator:

    def __init__(self, lat, lng, zoom=12):
        self._lng = lng
        self._lat = lat
        self._zoom = zoom

        self._image_origin_x = 0
        self._image_origin_y = 0

    def getXY(self, lat, lng):
        limit_lat = 85.05112878

        point_x = 2**(self._zoom + 7) * (lng/180 + 1)
        point_y = 2**(self._zoom + 7)/math.pi * (- math.atanh(math.sin(math.pi/180 * lat)) + math.atanh(math.sin(math.pi/180 * limit_lat)))

        return int(point_x), int(point_y)

    def get_point_on_image(self, lat, lng):
        point_x, point_y = self.getXY(lat, lng)

        point_x -= self._image_origin_x
        point_y -= self._image_origin_y
        return point_x, point_y

    def getGPS(self, point_x, point_y):
        limit_lat = 85.05112878

        point_x += self._image_origin_x
        point_y += self._image_origin_y

        lng = 180 * (point_x/2**(self._zoom + 7) - 1)
        lat = 180/math.pi * (math.asin(math.tanh(-math.pi/2**(self._zoom + 7) * point_y + math.atanh(math.sin(math.pi/180 * limit_lat)))))
 
        return lat, lng

    def generateImage(self, **kwargs):
        start_x = kwargs.get('start_x', None)
        start_y = kwargs.get('start_y', None)
        tile_width = kwargs.get('tile_width', 3)
        tile_height = kwargs.get('tile_height', 3)

        # Check that we have x and y tile coordinates
        if start_x == None or start_y == None:
            start_x, start_y = self.getXY(self._lat, self._lng)
            start_x = int(start_x/256) - 1
            start_y = int(start_y/256) - 1

            self._image_origin_x = start_x * 256
            self._image_origin_y = start_y * 256

        # Determine the size of the image
        width, height = 256 * tile_width, 256 * tile_height

        # Create a new image of the size require
        map_img = Image.new('RGB', (width, height))

        for x in range(0, tile_width):
            for y in range(0, tile_height):
                url = 'https://mt0.google.com/vt?x=' + str(start_x + x) + '&y=' + str(start_y + y) + '&z=' + str(
                    self._zoom)

                current_tile = str(x) + '-' + str(y)
                urllib.request.urlretrieve(url, current_tile)

                im = Image.open(current_tile)
                map_img.paste(im, (x * 256, y * 256))

                os.remove(current_tile)

        return map_img
