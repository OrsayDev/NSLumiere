# object that is used to describe scan tables both for scan and detector devices.

import gettext
import enum
import typing
from ctypes import cdll, create_string_buffer, POINTER, byref
from ctypes import c_uint, c_int, c_char, c_char_p, c_void_p, c_short, c_long, c_bool, c_double, c_uint64, c_uint32, \
    Array

import numpy
import math

_ = gettext.gettext

__author__ = "Marcel Tencé"
__status__ = "alpha"
__version__ = "0.1"

class ScanPatternShapes(enum.IntEnum):
    DEFAULT = 0
    TRIANGLE = 1
    TRANSPOSED = 2
    SUBSQUARES = 3


class ScanPattern(object):

    def __build_function(self, str_call, args, result):
        if hasattr(self.__scan_dll, str_call):
            call = self.__scan_dll.__getattr__(str_call)
            call.argtypes = args
            call.restype = result
            return call
        else:
            raise Exception("*** " + str_call + " function not found in Orsay scan.dll ***")



    def __initialize_library(self, library) -> None:
        self.__scan_dll = library
        # void ScanPatternAPI *OrsayScanPatternInit();
        self.__Init = self.__build_function("OrsayScanPatternInit", None, c_void_p)
        # void ScanPatternAPI OrsayScanPatternClose(void *o);
        self.__Close = self.__build_function("OrsayScanPatternClose", [c_void_p], None)

        # enum ScanPatternAPI OrsayScanPatternShapes { DEFAULT = 0, TRIANGLE = 1, TRANSPOSED = 2, SUBSQUARES = 3 };

        # bool ScanPatternAPI OrsayScanLoadXYArray(void *o, int sizex, int sizey, int nbpoints, int* xyarray);
        self.__LoadXYArray = self.__build_function("OrsayScanLoadXYArray", [c_void_p, c_int, c_int, c_int, POINTER(c_int)], c_bool)

        # 	bool ScanPatternAPI OrsayScanGetXYArray(void *o, int points, int *xyarray);
        self.__GetXYArray = self.__build_function("OrsayScanGetXYArray", [c_void_p, c_int, POINTER(c_int)], c_bool)

        # void ScanPatternAPI OrsayScanGetPatternDimensions(void *o, int &sx, int &sy, int &points);
        self.__GetPatternDimensions = self.__build_function("OrsayScanGetPatternDimensions",
                                    [c_void_p, POINTER(c_int), POINTER(c_int), POINTER(c_int)], None)

        # bool ScanPatternAPI OrsayScanGenerateXYArray(void *o, int sx, int sy, int startx, int starty, int nbpoints, int shape, bool shuffle);
        self.__GenerateXYArray = self.__build_function("OrsayScanGenerateXYArray",
                               [c_void_p, c_int, c_int, c_int, c_int, c_int, c_int, c_bool], c_bool)

        # bool ScanPatternAPI OrsayScanReGenerateXYArray(void *o, int sx, int sy, int startx, int starty, int nbpoints);
        self.__ReGenerateXYArray = self.__build_function("OrsayScanReGenerateXYArray",
                                                  [c_void_p, c_int, c_int, c_int, c_int, c_int], c_bool)

        # 	void ScanPatternAPI SetListPixelStart(void * o, int mode);
        self.__SetListPixelStart = self.__build_function("SetListPixelStart", [c_void_p, c_int], None)
        # 	int ScanPatternAPI GetListPixelStart(void * o);
        self.__GetListPixelStart = self.__build_function("SetListPixelStart", [c_void_p], c_int)

    """
    Create a scan pattern object using the following libray
    :param library_name:
    """

    def __init__(self, library):
        self.__pixel_delay = 0
        self.__initialize_library(library)
        self.pattern = self.__Init()
        self.size_x = 512
        self.size_y = 512
        self.num_of_points = 0

    """
    Destroy scan pattern object
    :return:
    """

    def close(self) -> None:
        self.__Close(self.pattern)

    """
    Give the enclosing array of the pattern and the num of points in the pattern.
    :return:
    """

    @property
    def shape(self):
        size_x = c_int()
        size_y = c_int()
        num_of_points = c_int()
        self.__GetPatternDimensions(self.pattern, size_x, size_y, num_of_points)
        self.num_of_points = num_of_points.value
        return {"size_x": size_x.value, "size_y": size_y.value, "num_of_points": num_of_points.value}

    """
    Write a scan table. All points[x,y] must be ine the range [0, size_x-1], [0, size_y-1]
    :param size_x:
    :param size_y:
    :param num_of_points:
    :param points: numpy array: list of positions.
    :return: num of points taken.
    """

    def write_xy_array(self, size_x, size_y, num_of_points, points) -> int:
        if self.__LoadXYArray(self.pattern, size_x, size_y, num_of_points,
                                                points.ctypes.data_as(POINTER(c_int))):
            self.size_x = size_x
            self.size_y = size_y
            self.num_of_points = num_of_points
        return self.num_of_points

    """
    Read the scan table array
    :param points: array that will receive the table
    :param size: size of the array. Must be greater or equal to the scan table size
    :return:
    enclosing array shap and size of the scan table copied
    """

    def read_xy_array(self):
        num_of_points = self.shape["num_of_points"]
        points = numpy.zeros(num_of_points, dtype=int)
        nb_points = self.__GetXYArray(self.pattern, num_of_points, points.ctypes.data_as(POINTER(c_int)))
        if nb_points:
            return points
        else:
            return None

    """
    Automatically generate a rectangular scan pattern
    :param size_x:
    :param size_y:
    :param start_x:
    :param start_y:
    :param num_of_points:
    :param shape:
    :param shuffle:
    :return:
    """
    def generate_xy_array(self, size_x: int, size_y: int, start_x: int, start_y: int,
                          num_of_points: int, shape: ScanPatternShapes, shuffle: bool) -> bool:
        return self.__GenerateXYArray(self.pattern, size_x, size_y, start_x, start_y, num_of_points, shape, shuffle)

    def regenerate_xy_array(self, size_x, size_y, start_x, start_y, num_of_points) -> bool:
        return self.__ReGenerateXYArray(self.pattern, size_x, size_y, start_x, start_y, num_of_points)

    """
    defines the trigger generated by the scan list
    0: a triger at every pixel
    1: a trigger only when bit 30 is set in the pixel value.
    """
    @property
    def list_pixel_start(self) -> int:
        return self.__GetListPixelStart(self.pattern)

    @list_pixel_start.setter
    def list_pixel_start(self, value:int) -> None:
        self.__SetListPixelStart(self.pattern, value)

    def draw_circle(self, size_x: float, size_y: float, center_x: float, center_y: float, radius: float, step: float) -> int():
        # change 0.95 to limit the arc size of the circle 1, full circle, 0.5 half circle ...
        num_of_points = int(2*math.pi/step * 1)
        points = numpy.zeros(num_of_points, dtype=int)
        for i in range(0, num_of_points):
            dx = center_x + radius * math.cos(i*step)
            dy = center_y + radius * math.sin(i*step)
            points[i] = int(dy) * size_x + int(dx)
        return points

    def draw_rectangle_filled(self, size_x: int, size_y: int, transposed: bool) -> int():
        num_of_points = size_x * size_y
        points = numpy.zeros(num_of_points, dtype=int)
        k = 0
        if transposed:
            for i in range(size_x):
                for j in range(size_y):
                    points[k] = j * size_x + i
                    k = k+1
        else:
            for j in range(size_y):
                for i in range(size_x):
                    points[k] = j * size_x + i
                    k = k+1
        return points

    def draw_rectangle_filled_subareas(self, size_x: int, size_y: int, areas_x: int, areas_y: int, transposed: bool) -> int():
        num_of_points = size_x * size_y
        points = numpy.zeros(num_of_points, dtype=int)
        k = 0
        area_x = size_x // areas_x;
        area_y = size_y // areas_y;
        for sqy in range(areas_y):
            offset_y = sqy * area_y
            area_size_y = area_y
            if (sqy > 1) and (sqy == areas_y - 1):
                area_size_y = size_y - sqy * area_y
            for sqx in range(areas_x):
                offset_x = sqx * area_x
                area_size_x = area_x
                if (sqx > 1) and (sqx == areas_x - 1):
                    area_size_x = size_x - sqx * area_x
                transposed = sqx % 2 != 0
                if transposed:
                    for i in range(area_size_x):
                        for j in range(area_size_y):
                            points[k] = (j + sqy * area_size_y) * size_x + (i + sqx * area_size_x)
                            k = k+1
                else:
                    for j in range(area_size_y):
                        for i in range(area_size_x):
                            points[k] = (j + sqy * area_size_y) * size_x + (i + sqx * area_size_x)
                            k = k+1
        return points

    def draw_spiral(self, size_x: int, size_y: int) -> int():
        num_of_points = size_x * size_y
        points = numpy.zeros(num_of_points, dtype=int)
        k = 0
        x_start = 0
        x_end = size_x
        # x_actual = start_x
        y_start = 0
        y_end = size_y
        y_actual = 0
        while k < num_of_points:
            # first path top-left, top-right
            # print(f"1st path x:[{x_start} -> {x_end - 1}]  y:{y_actual}")
            for i in range(x_start, x_end):
                points[k] = i + y_actual * size_x
                k = k+1
            if k >= num_of_points:
                break
            # print(f"nombre de points {k}")
            x_actual = x_end - 1
            y_start = y_start + 1
            # second path top-right, bottom-right
            # print(f"2nd path x:{x_actual} y:[{y_start} ->  {y_end - 1}]")
            for j in range(y_start, y_end):
                points[k] = x_actual + j * size_x
                k = k+1
            if k >= num_of_points:
                break
            # print(f"nombre de points {k}")
            x_end = x_end - 1
            y_actual = y_end - 1
            # third path bottom-right, bottom-left
            # print(f"3rd path x:[{x_end - 1} -> {x_start}]  y:{y_actual}")
            for i in range(x_end - 1, x_start - 1, -1):
                points[k] = i + y_actual * size_x
                k = k+1
            if k >= num_of_points:
                break
            # print(f"nombre de points {k}")
            y_end = y_end - 1
            x_actual = x_start
            # fourth path bottom-left, top-left
            # print(f"4th path x:{x_actual} y:[{y_end} ->  {y_start}]")
            for j in range(y_end - 1, y_start - 1, -1):
                points[k] = x_actual + j * size_x
                k = k+1
            if k >= num_of_points:
                break
            # print(f"nombre de points {k}")
            y_actual = y_start
            x_start = x_start + 1
        # print(f"nombre de points {k} expected {num_of_points}")
        return points
