"""
Demo program for class controlling orsay scan hardware.
"""
# standard libraries
import math
import sys
from ctypes import c_void_p, POINTER, c_int

import numpy
from _ctypes import byref, POINTER
import numpy as np
from scan_pattern import ScanPattern, ScanPatternShapes

import orsayscanM as orsayscan
#from nionswift_plugin.IVG.scan import orsayscan
# import orsayscanVG


def testgeneimage(scan):
    sizex = 512
    sizey = 512
    sizez = 1
    imagedata = None

    def dataLocker(gene, datatype, sx, sy, sz):
        sx[0] = sizex
        sy[0] = sizey
        sz[0] = sizez
        datatype[0] = 2
        pointeur = imagedata.ctypes.data_as(c_void_p)
        return pointeur.value

    def dataUnlocker(gene, newdata):
        print(imagedata[0:16])

    def dataUnlockerA(gene, newdata, imagenb, rect):
        if newdata:
            print(
                f"Py Image: {gene}    image nb: {imagenb}   pos: [{rect[0]}, {rect[1]}]   size: [{rect[2]}, {rect[3]}]")
            #    print ("Scan count: ", orsayscan.getScanCount())

    scan.externalclock = (0.0, 0.001)
    clk = scan.externalclock

    nbinputs = scan.getInputsCount()
    k = 0
    while (k < nbinputs):
        unipolar, offset, name, dd = scan.getInputProperties(k)
        print(f"Input: {k}   label: {name}   video offset: {offset}")
        k = k + 1
    scan.setScanClock(-1)
    # choose X and Y ramps.
    scan.SetInputs([6, 7])
    sizez, inputs = scan.GetInputs()
    scan.setImageSize(sizex, sizey)
    sizex, sizey = scan.getImageSize()
    scan.setImageArea(sizex, sizey, 0, sizex, 0, sizey)
    res, sx, sy, stx, ex, sty, ey = scan.getImageArea()
    print(f"Pixel time: {scan.pixelTime}")
    scan.pixelTime = 0.000002
    print(f"Pixel time: {scan.pixelTime}")

    fb = scan.SetFlybackTime(0.000123)
    fb1 = scan.GetFlybackTime()
    imagedata = np.ones((sizez, sizey, sizex), dtype=np.uint16)

    fnlock = orsayscan.LOCKERFUNC(dataLocker)
    scan.registerLocker(fnlock)

    fnunlock = orsayscan.UNLOCKERFUNCA(dataUnlockerA)
    scan.registerUnlockerA(fnunlock)

    input("Taper un caractère pour démarrer le scan, puis un autre pour l'arrêter")
    scan.startImaging(0, 1)
    input("")

    scan.stopImaging(1)


def testgenespim(spimscan):
    sizex = 512
    sizey = 512
    sizez = 1
    imagedata = None

    def spimdataLocker(gene, datatype, sx, sy, sz):
        sx[0] = sizex
        sy[0] = sizey
        sz[0] = sizez
        datatype[0] = 2
        pointeur = imagedata.ctypes.data_as(c_void_p)
        return pointeur.value

    def spimdataUnlockerA(gene, newdata, imagenb, rect):
        if newdata:
            print(
                f"Py Image: {gene}    image nb: {imagenb}   pos: [{rect[0]}, {rect[1]}]   size: [{rect[2]}, {rect[3]}]")
            #    print ("Scan count: ", orsayscan.getScanCount())

    fnlock = orsayscan.LOCKERFUNC(spimdataLocker)
    spimscan.registerLocker(fnlock)

    fnunlock = orsayscan.UNLOCKERFUNCA(spimdataUnlockerA)
    spimscan.registerUnlockerA(fnunlock)

    nbinputs = spimscan.getInputsCount()
    k = 0
    while (k < nbinputs):
        unipolar, offset, name, dd = spimscan.getInputProperties(k)
        print(f"Input: {k}   label: {name}   video offset: {offset}")
        k = k + 1
    print(f"Pixel time: {spimscan.pixelTime}")
    spimscan.SetInputs([7])
    sizez, inputs = spimscan.GetInputs()
    spimscan.setImageArea(sizex, sizey, 0, sizex, 0, sizey)
    sizex, sizey = spimscan.getImageSize()
    res, sx, sy, stx, ex, sty, ey = spimscan.getImageArea()
    # test sur l'entrée eels. (cl = 4)
    spimscan.setScanClock(2)
    #
    # simulatation clock camera (tester si le mode simulation est activé sur la caméra et retrouver les 2 paramètres suivants:
    camera_simul = True
    camera_readout = 0.002
    camera_exposure = 0.001
    if camera_simul:
        spimscan.clock_simulation_time = camera_readout + camera_exposure
    else:
        spimscan.clock_simulation_time = 0

    spimscan.pixelTime = camera_exposure
    input("Taper un caractère pour démarrer le scan, puis un autre pour l'arrêter")
    imagedata = np.ones((sizez, sizey, sizex), dtype=np.uint16)
    spimscan.startImaging(0, 1)
    input("")
    spimscan.stopImaging(1)

def draw_circle(size_x, size_y, center_x, center_y, radius, step):
    # change 0.95 to limit the arc size of the circle 1, full circle, 0.5 half circle ...
    ang = 1.0
    num_of_points = int(2*math.pi/step * ang)
    points = numpy.zeros(num_of_points, dtype=int)
    dxmax = size_x - 1
    dxmin = 0
    dymax = size_y - 1
    dymin = 0
    for i in range(0, num_of_points):
        dx = int(center_x + radius * math.cos(i*step))
        dy = int(center_y + radius * math.sin(i*step))
        if (dx > dxmax): dx = dxmax
        if (dx < dxmin): dx = dxmin
        if (dy > dymax): dy = dymax
        if (dy < dymin): dy = dymin
        points[i] = int(dy) * size_x + int(dx)
    return points

def test_table_scan(scan: orsayscan.orsayScan, efm03:bool):
    #number of points in the circle
    nbpts = 5
    # max image size 8192x8192 at the moment
    image_size = 1024
    centre = int(image_size/2)
    radius = int(image_size/2 * 1)
    points = draw_circle(image_size, image_size, centre, centre, radius, 2 * math.pi / nbpts)
    # add start of pattern trigger bit (bit 31 here). A trigger bit can be inserted anywhere in the list
    # when blanker required inside the list bit 15 can set to 1.
    mask_pixel = numpy.iinfo(numpy.int32).min
    mask_blanker = 32768
    points[0] = points[0] | mask_pixel
    points[points.size // 2] = points[points.size // 2] | mask_pixel
    # load points table into the scan pattern class
    points_writen = scan.scan_pattern.write_xy_array(image_size, image_size, points.size, points)
    # retrieve the shape the pattern is in
    shape = scan.scan_pattern.shape
    # select trigger:  0 every pixel, 1: as defined in bit 30 of each pixel point.
    #
    scan.scan_pattern.list_pixel_start = 1
    # pixel time in seconds
    scan.pixelTime = 0.000005
    # define the shape the pulse that will occur on every pixel
    # a 1 µs pulse signal when trigger bit is there (or new pixel)
    if efm03:
        scan.enable_lines(0x3FF)
        scan.set_output_line_a(4, mode=3, source=5, period=0, on_time=1.0e-6, nb_pulses=0, filtered=False)
    else:
        scan.set_output_line_a(4, mode=3, source=5, period=0, on_time=1.0e-6, nb_pulses=0, filtered=False)
    # ready now start the scan.
    scan.startTableImaging(1, scan.scan_pattern.pattern, numpy.iinfo(numpy.uint64).max)
    # read back the pattern
    points_read = scan.scan_pattern.read_xy_array()
    # end of application, scan is running indefinitely (almost)
    scan.scan_pattern.close()

if __name__ == "__main__":
    efm03 = True
    scan = orsayscan.orsayScan(1, efm03 = efm03)
    spimscan = orsayscan.orsayScan(2, scan.orsayscan)
    # testgeneimage(scan)
    test_table_scan(scan, efm03)

    # testgenespim(spimscan)