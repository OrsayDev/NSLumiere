# standard libraries
import copy
import math
import ctypes
import gettext
import numpy
import threading
import typing
import logging
import os
import json
import time

# local libraries
from nion.utils import Registry
from nion.utils import Geometry
from nion.instrumentation import scan_base
from nion.instrumentation import stem_controller
from nion.swift.model import HardwareSource

from nionswift_plugin.IVG.scan.orsayscan import orsayScan, LOCKERFUNC, UNLOCKERFUNCA
from nionswift_plugin.IVG.scan.ConfigVGLumDialog import ConfigDialog

from nionswift_plugin.IVG import ivg_inst

_ = gettext.gettext


# set the calibrations for this image. does not touch metadata.
def test_update_scan_data_element(data_element, scan_frame_parameters, data_shape, channel_name, channel_id,
                             scan_properties):
    scan_properties = copy.deepcopy(scan_properties)
    pixel_time_us = float(scan_properties["pixel_time_us"])
    line_time_us = float(scan_properties["line_time_us"]) if "line_time_us" in scan_properties else pixel_time_us * \
                                                                                                    data_shape[1]
    center_x_nm = float(scan_properties.get("center_x_nm", 0.0))
    center_y_nm = float(scan_properties.get("center_y_nm", 0.0))
    fov_nm = float(scan_frame_parameters["fov_nm"])  # context fov_nm, not actual fov_nm returned from low level
    if scan_frame_parameters.size[0] > scan_frame_parameters.size[1]:
        fractional_size = scan_frame_parameters.subscan_fractional_size[
            0] if scan_frame_parameters.subscan_fractional_size else 1.0
        pixel_size = scan_frame_parameters.subscan_pixel_size[0] if scan_frame_parameters.subscan_pixel_size else \
        scan_frame_parameters.size[0]
        pixel_size_nm = fov_nm * fractional_size / pixel_size
    else:
        fractional_size = scan_frame_parameters.subscan_fractional_size[
            1] if scan_frame_parameters.subscan_fractional_size else 1.0
        pixel_size = scan_frame_parameters.subscan_pixel_size[1] if scan_frame_parameters.subscan_pixel_size else \
        scan_frame_parameters.size[1]
        pixel_size_nm = fov_nm * fractional_size / pixel_size
    data_element["title"] = channel_name
    data_element["version"] = 1
    data_element["channel_id"] = channel_id  # needed to match to the channel
    data_element["channel_name"] = channel_name  # needed to match to the channel
    if scan_properties.get("calibration_style") == "time":
        data_element["spatial_calibrations"] = (
            {"offset": 0.0, "scale": line_time_us / 1E6, "units": "s"},
            {"offset": 0.0, "scale": pixel_time_us / 1E6, "units": "s"}
        )
    else:
        data_element["spatial_calibrations"] = [
            {"offset": -center_y_nm - pixel_size_nm * data_shape[0] * 0.5, "scale": pixel_size_nm, "units": "nm"},
            {"offset": -center_x_nm - pixel_size_nm * data_shape[1] * 0.5, "scale": pixel_size_nm, "units": "nm"}
        ]
        if len(data_shape) == 3: #3D acquisitions. This is typically a hyperspectral image.
            eels_dispersion = float(scan_properties.get("eels_dispersion", 1.0))
            eels_offset = float(scan_properties.get("eels_offset", 0.0))
            data_element["spatial_calibrations"].append(
                {"offset": eels_offset, "scale": eels_dispersion, "units": "eV"}
            )

#This is a very ugly thing apparently called MONKEY PATCHING. Although ugly, it is a perfectly fine solution
#scan_base.update_scan_data_element = test_update_scan_data_element


class Channel:
    def __init__(self, channel_id: int, name: str, enabled: bool):
        self.channel_id = channel_id
        self.name = name
        self.enabled = enabled
        self.data = None


class Frame:

    def __init__(self, frame_number: int, channels: typing.List[Channel],
                 frame_parameters: scan_base.ScanFrameParameters):
        self.frame_number = frame_number
        self.channels = channels
        self.frame_parameters = frame_parameters
        self.complete = False
        self.bad = False
        self.data_count = 0
        self.start_time = time.time()
        self.scan_data = None

class Device(scan_base.ScanDevice):

    def __init__(self, instrument: ivg_inst.ivgInstrument):
        self.scan_device_id = "orsay_scan_device"
        self.scan_device_name = _("VG")
        self.stem_controller_id = "VG_controller"
        self.__channels = self.__get_channels()
        self.__frame = None
        self.__frame_number = 0
        self.__instrument = instrument
        self.__sizez = 2
        self.__probe_position = [0, 0]
        self.__probe_position_pixels = [0, 0]
        self.__rotation = 0.
        self.__is_scanning = False
        self.on_device_state_changed = None
        self.__profiles = self.__get_initial_profiles()
        self.__frame_parameters = copy.deepcopy(self.__profiles[0])
        self.flyback_pixels = 2
        self.__buffer = list()
        self.bottom_blanker = 0

        self.orsayscan = orsayScan(1, vg=True)
        self.spimscan = orsayScan(2, self.orsayscan.orsayscan, vg=True)

        self.orsayscan.SetInputs([1, 0])
        self.spimscan.SetInputs([1, 0])

        self.has_data_event = threading.Event()

        self.fnlock = LOCKERFUNC(self.__data_locker)
        self.orsayscan.registerLocker(self.fnlock)
        self.fnunlock = UNLOCKERFUNCA(self.__data_unlockerA)
        self.orsayscan.registerUnlockerA(self.fnunlock)

        self.orsayscan.setScanScale(0, 5.0, 5.0)

        ######

        self.pixel_time = 0.5
        self.field_of_view = 4000
        self.orsayscan.SetProbeAt(0, 0)
        self.__spim_pixels = (0, 0)
        self.subscan_status = True
        self.__spim = False

        self.__tpx3_spim = False
        self.__tpx3_data = None
        self.__tpx3_camera = None
        self.__tpx3_calib = dict()
        self.__tpx3_frameStop = 0

        self.p0 = 512
        self.p1 = 512
        self.p2 = 0
        self.p3 = 512
        self.p4 = 0
        self.p5 = 512
        self.Image_area = [self.p0, self.p1, self.p2, self.p3, self.p4, self.p5]
        #self.set_spim_pixels = [self.p0, self.p1, self.p2, self.p3, self.p4, self.p5]
        self.orsayscan.setScanRotation(22.5)

        #Set HADF and BF initial gain values
        self.orsayscan.SetPMT(1, 2200)
        self.orsayscan.SetPMT(0, 2200)

        ######

        self.scan = self.orsayscan

    def close(self):
        pass

    def stop(self) -> None:
        """Stop acquiring."""
        self.__spim = False
        if self.__is_scanning:
            self.orsayscan.stopImaging(True)
            self.spimscan.stopImaging(True)
            self.stop_timepix3()
            self.__is_scanning = False

    def set_idle_position_by_percentage(self, x: float, y: float) -> None:
        """Set the idle position as a percentage of the last used frame parameters."""
        pass

    def cancel(self) -> None:
        """Cancel acquisition (immediate)."""
        self.__spim = False
        if self.__is_scanning:
            self.orsayscan.stopImaging(False)
            self.spimscan.stopImaging(True)
            self.stop_timepix3()
            self.__is_scanning = False

    def __get_channels(self) -> typing.List[Channel]:
        channels = [Channel(0, "ADF", True), Channel(1, "BF", False)]
        abs_path = os.path.abspath('C:\ProgramData\Microscope\Orsay_cameras_list.json')
        try:
            with open(abs_path) as savfile:
                cameras = json.load(savfile)
        except FileNotFoundError:
            abs_path = os.path.join(os.path.dirname(__file__), '../../aux_files/config/Orsay_cameras_list.json')
            with open(abs_path) as savfile:
                cameras = json.load(savfile)
        for camera in cameras:
            if camera["manufacturer"] == 4:
                channels.append(Channel(2, "TPX3", False))
        return channels

    def __get_initial_profiles(self) -> typing.List[scan_base.ScanFrameParameters]:
        profiles = list()
        profiles.append(scan_base.ScanFrameParameters({"size": (512, 512), "pixel_time_us": 0.5, "fov_nm": 4000., "rotation_rad": 0.393}))
        profiles.append(scan_base.ScanFrameParameters({"size": (128, 128), "pixel_time_us": 1, "fov_nm": 100.}))
        profiles.append(scan_base.ScanFrameParameters({"size": (512, 512), "pixel_time_us": 1, "fov_nm": 100.}))
        return profiles

    def get_profile_frame_parameters(self, profile_index: int) -> scan_base.ScanFrameParameters:
        return copy.deepcopy(self.__profiles[profile_index])

    def set_profile_frame_parameters(self, profile_index: int, frame_parameters: scan_base.ScanFrameParameters) -> None:
        """Set the acquisition parameters for the give profile_index (0, 1, 2)."""
        self.__profiles[profile_index] = copy.deepcopy(frame_parameters)

    def get_channel_name(self, channel_index: int) -> str:
        return self.__channels[channel_index].name

    def set_frame_parameters(self, frame_parameters: scan_base.ScanFrameParameters) -> None:
        """Called just before and during acquisition.
        Device should use these parameters for new acquisition; and update to these parameters during acquisition.
        """
        self.__frame_parameters = copy.deepcopy(frame_parameters)
        if self.field_of_view != frame_parameters.fov_nm: self.field_of_view = frame_parameters.fov_nm
        if self.pixel_time != frame_parameters.pixel_time_us: self.pixel_time = frame_parameters.pixel_time_us

        if self.scan_rotation != frame_parameters.rotation_rad:
            self.scan_rotation = frame_parameters.rotation_rad

        if not self.__spim:
            if frame_parameters.subscan_pixel_size:
                self.subscan_status = True
                #self.__instrument.is_subscan_f=[True, frame_parameters['subscan_pixel_size'][1]/self.p0, frame_parameters['subscan_pixel_size'][0]/self.p1]
                self.p0, self.p1 = frame_parameters.size
                self.p2 = int(
                    frame_parameters.subscan_fractional_center[1] * self.p0 - frame_parameters.subscan_pixel_size[
                        1] / 2)
                self.p4 = int(
                    frame_parameters.subscan_fractional_center[0] * self.p1 - frame_parameters.subscan_pixel_size[
                        0] / 2)
                self.p3 = self.p2 + frame_parameters.subscan_pixel_size[1]
                self.p5 = self.p4 + frame_parameters.subscan_pixel_size[0]
                #subscan = frame_parameters['subscan_pixel_size']
                #logging.info(f'***SCAN***: Setting subscan to {subscan}.')
                self.Image_area = [self.p0, self.p1, self.p2, self.p3, self.p4, self.p5]
            else:
                self.subscan_status = False
                #if self.subscan_status or (self.Image_area[0], self.Image_area[1]) != frame_parameters['size']:
                self.p0, self.p1 = frame_parameters.size
                self.p2, self.p4 = 0, 0
                self.p3, self.p5 = self.p0, self.p1
                self.Image_area = [self.p0, self.p1, 0, self.p3, 0, self.p5]

                #self.__instrument.is_subscan_f=[False, 1, 1]

    def save_frame_parameters(self) -> None:
        """Called when shutting down. Save frame parameters to persistent storage."""
        pass

    def prepare_timepix3(self):
        self.__tpx3_camera = HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id(
            "orsay_camera_timepix3")
        self.__tpx3_spim = self.__tpx3_camera.camera.camera.StartSpimFromScan()
        if self.__tpx3_spim: #True if successful
            self.__tpx3_calib["dispersion"] = self.__instrument.TryGetVal("eels_x_scale")[1]
            self.__tpx3_calib["offset"] = self.__instrument.TryGetVal("eels_x_offset")[1]
            self.__tpx3_camera.camera.camera._TimePix3__isReady.wait(5.0)
            self.__tpx3_data = self.__tpx3_camera.camera.camera.create_spimimage_from_events()
            time.sleep(0.5) #Timepix has already a socket connected. Wait until it is definitely reading data

    def stop_timepix3(self):
        if self.__tpx3_spim: #only stop if this was on
            self.__tpx3_camera = HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id(
            "orsay_camera_timepix3")
            self.__tpx3_camera.camera.camera.stopSpim(True)
            self.__tpx3_spim = False

    def no_prepare_timepix3(self):
        self.__tpx3_data = None

    def start_frame(self, is_continuous: bool) -> int:
        """Start acquiring. Return the frame number."""
        if not self.__is_scanning:
            self.__buffer = list()
            self.__start_next_frame()

            logging.info(f"***SCAN***: Starting acquisition. Spim is {self.__spim}")

            # Picking between scans
            if self.__spim:
                self.spimscan.setScanClock(4)
                scan = self.spimscan
            else:
                scan = self.orsayscan

            # Selecting the proper imagedata
            self.__scan_size = scan.getImageSize()
            self.__sizez = scan.GetInputs()[0]
            if self.__sizez % 2:
                self.__sizez += 1

            self.imagedata = numpy.zeros((self.__sizez * (self.__scan_size[0]), (self.__scan_size[1])), dtype=numpy.int16)
            self.imagedata_ptr = self.imagedata.ctypes.data_as(ctypes.c_void_p)

            print(f'{self.__sizez} and {self.__scan_size} and {self.__scan_area} amd {self.Spim_image_area} and '
                  f'{self.Image_area} and {self.__spim_pixels} and {self.imagedata.shape} and {self.spimscan.getImageArea()} and {self.orsayscan.getImageArea()}')


            if not self.__spim:
                for channel in self.__channels:
                    if channel.name == 'TPX3':
                        if channel.enabled:
                            self.prepare_timepix3()
                        else:
                            self.no_prepare_timepix3()
                #Scan must be started after timepix3 so we are ready for receiving TDC's
                self.__is_scanning = self.orsayscan.startImaging(0, 1)
            else:
                self.__is_scanning = self.spimscan.startSpim(0, 1)

            logging.info(f'**SCAN***: Acquisition Started is {self.__is_scanning}.')
        return self.__frame_number

    def __start_next_frame(self):
        frame_parameters = copy.deepcopy(self.__frame_parameters)
        self.__scan_context = stem_controller.ScanContext()
        channels = [copy.deepcopy(channel) for channel in self.__channels if channel.enabled]  # channel enabled is here
        size = Geometry.IntSize.make(
            frame_parameters.subscan_pixel_size if frame_parameters.subscan_pixel_size else frame_parameters.size)
        for channel in channels:
            channel.data = numpy.zeros(tuple(size), numpy.float32)
        self.__frame_number += 1 #This is updated in the self.__frame_number
        self.__frame = Frame(self.__frame_number, channels, frame_parameters)

    def read_partial(self, frame_number, pixels_to_skip) -> (typing.Sequence[dict], bool, bool, tuple, int, int):
        """Read or continue reading a frame.
        The `frame_number` may be None, in which case a new frame should be read.
        The `frame_number` otherwise specifies which frame to continue reading.
        The `pixels_to_skip` specifies where to start reading the frame, if it is a continuation.
        Return values should be a list of dict's (one for each active channel) containing two keys: 'data' and
        'properties' (see below), followed by a boolean indicating whether the frame is complete, a boolean indicating
        whether the frame was bad, a tuple of the form (top, left), (height, width) indicating the valid sub-area
        of the data, the frame number, and the pixels to skip next time around if the frame is not complete.
        The 'data' keys in the list of dict's should contain a ndarray with the size of the full acquisition and each
        ndarray should be the same size. The 'properties' keys are dicts which must contain the frame parameters and
        a 'channel_id' indicating the index of the channel (may be an int or float).
        """

        gotit = self.has_data_event.wait(1.0)

        if self.__frame is None:
            self.__start_next_frame()

        current_frame = self.__frame  # this is from Frame Class defined above
        assert current_frame is not None
        frame_parameters = current_frame.frame_parameters
        is_synchronized_scan = frame_parameters.get_parameter("external_clock_mode", 0)
        scan_area = [self.Spim_image_area[0], self.Spim_image_area[1]] if is_synchronized_scan else\
            [self.Image_area[3]-self.Image_area[2], self.Image_area[5]-self.Image_area[4]]
        scan_offset = [0, 0] if is_synchronized_scan else [self.Image_area[2], self.Image_area[4]]
        #scan_area = self.Spim_image_area if is_synchronized_scan else self.Image_area

        if self.__line_number == scan_area[1]:
            current_frame.complete = True
            pixels_to_skip = 0
        else:
            pixels_to_skip = self.__line_number * self.__scan_size[1]

        data_elements = list()

        for channel in current_frame.channels:
            data_element = dict()

            #Timepix3 Spim channel
            if channel.name == 'TPX3':
                if self.__frame_number % 10 == 0:
                    data_array = self.__tpx3_data
                    data_element["data"] = data_array
                    properties = current_frame.frame_parameters.as_dict()
                    properties["center_x_nm"] = current_frame.frame_parameters.center_nm[1]
                    properties["center_y_nm"] = current_frame.frame_parameters.center_nm[0]
                    properties["rotation_deg"] = math.degrees(current_frame.frame_parameters.rotation_rad)
                    properties["channel_id"] = channel.channel_id
                    properties["eels_dispersion"] = self.__tpx3_calib["dispersion"]
                    properties["eels_offset"] = self.__tpx3_calib["offset"]
                    data_element["properties"] = properties
                    if data_array is not None:
                        data_elements.append(data_element)

            else:
                data_array = self.imagedata[channel.channel_id * (scan_area[1]) + scan_offset[1]:(channel.channel_id + 1) * (
                    scan_area[1]) + scan_offset[1], scan_offset[0]: (scan_area[0]) + scan_offset[0]].astype(numpy.float32)
                data_element["data"] = data_array
                properties = current_frame.frame_parameters.as_dict()
                sub_area = ((0, 0), data_array.shape)
                properties['sub_area'] = ((0, 0), data_array.shape)
                properties["center_x_nm"] = current_frame.frame_parameters.center_nm[1]
                properties["center_y_nm"] = current_frame.frame_parameters.center_nm[0]
                properties["rotation_deg"] = math.degrees(current_frame.frame_parameters.rotation_rad)
                properties["channel_id"] = channel.channel_id
                data_element["properties"] = properties
                if data_array is not None:
                    data_elements.append(data_element)


                # if not self.__spim:
                # #if not self.__spim and self.__isplaying:
                #     data_array = self.imagedata[channel.channel_id * (self.__scan_area[1]):(channel.channel_id + 1) * (
                #     self.__scan_area[1]),
                #                  0+1: (self.__scan_area[0]-1)]
                #     if self.subscan_status:  # Marcel programs returns 0 pixels without the sub scan region so i just crop
                #         data_array = data_array[self.p4:self.p5, self.p2:self.p3]
                #     data_element["data"] = data_array
                #     properties = current_frame.frame_parameters.as_dict()
                #     properties["center_x_nm"] = current_frame.frame_parameters.center_nm[1]
                #     properties["center_y_nm"] = current_frame.frame_parameters.center_nm[0]
                #     properties["rotation_deg"] = math.degrees(current_frame.frame_parameters.rotation_rad)
                #     properties["channel_id"] = channel.channel_id
                #     data_element["properties"] = properties
                #     if data_array is not None:
                #         data_elements.append(data_element)
                #
                # elif self.__spim:
                #     data_array = self.imagedata[channel.channel_id * (self.__scan_area[1]):channel.channel_id * (
                #         self.__scan_area[1]) + self.__spim_pixels[1],
                #                  0: (self.__spim_pixels[0])].astype(numpy.float32)
                #     #data_array = self.imagedata.astype(numpy.float32)
                #     #if self.subscan_status:  # Marcel programs returns 0 pixels without the sub scan region so i just crop
                #     #    data_array = data_array[self.p4:self.p5, self.p2:self.p3]
                #     data_element["data"] = data_array
                #     properties = current_frame.frame_parameters.as_dict()
                #     properties["center_x_nm"] = current_frame.frame_parameters.center_nm[1]
                #     properties["center_y_nm"] = current_frame.frame_parameters.center_nm[0]
                #     properties["rotation_deg"] = math.degrees(current_frame.frame_parameters.rotation_rad)
                #     properties["channel_id"] = channel.channel_id
                #     data_element["properties"] = properties
                #     if data_array is not None:
                #         data_elements.append(data_element)

        bad_frame = False
        self.has_data_event.clear()

        #current_frame.complete = True
        if current_frame.complete:
            self.__frame = None

        #Stop by using the number of frames
        if self.__tpx3_spim and self.__tpx3_frameStop != 0 and self.__frame_number >= self.__tpx3_frameStop:
            logging.info(f"***SCAN***: Stopped frame acquisition by using the frame stop property. Total number"
                         f"of frames are {self.__frame_number}.")
            self.stop()




        # return data_elements, complete, bad_frame, sub_area, frame_number, pixels_to_skip
        return data_elements, current_frame.complete, False, ((0, 0), data_array.shape), None, pixels_to_skip
        #return data_elements, current_frame.complete, bad_frame, sub_area, self.__frame_number, pixels_to_skip*

    #This one is called in scan_base
    def prepare_synchronized_scan(self, scan_frame_parameters: scan_base.ScanFrameParameters, *, camera_exposure_ms, **kwargs) -> None:
        scan_frame_parameters["pixel_time_us"] = min(5120000, int(1000 * camera_exposure_ms * 0.75))
        scan_frame_parameters["external_clock_wait_time_ms"] = 20000 # int(camera_frame_parameters["exposure_ms"] * 1.5)
        scan_frame_parameters["external_clock_mode"] = 1

        if scan_frame_parameters['subscan_pixel_size']:
            size = size = scan_frame_parameters['subscan_pixel_size']
        else:
            size = scan_frame_parameters["size"]

        self.Spim_image_area = [size[0], size[1]]
        self.__spim = True

    def calculate_flyback_pixels(self, frame_parameters: scan_base.ScanFrameParameters) -> int:
        return 0

    def set_channel_enabled(self, channel_index: int, enabled: bool) -> bool:
        assert 0 <= channel_index < self.channel_count
        self.__channels[channel_index].enabled = enabled
        if not any(channel.enabled for channel in self.__channels):
            self.cancel()
        return True

    def set_scan_context_probe_position(self, scan_context: stem_controller.ScanContext,
                                        probe_position: Geometry.FloatPoint):
        if probe_position:
            self.probe_pos = probe_position

    @property
    def field_of_view(self):
        return self.__fov

    @field_of_view.setter
    def field_of_view(self, value):
        self.__fov = value / 1e9
        if self.__fov > 72 * 1e-6: self.__fov = 72. * 1e6
        self.orsayscan.SetFieldSize(self.__fov)
        self.__instrument.fov_change(self.__fov)

    @property
    def pixel_time(self):
        return self.__pixeltime

    @pixel_time.setter
    def pixel_time(self, value):
        self.__pixeltime = value / 1e6
        self.orsayscan.pixelTime = self.__pixeltime

    @property
    def scan_rotation(self):
        return self.__rotation

    @scan_rotation.setter
    def scan_rotation(self, value):
        self.__rotation = value*180/numpy.pi
        self.orsayscan.setScanRotation(self.__rotation)

    @property
    def Image_area(self):
        return self.__scan_area

    @Image_area.setter
    def Image_area(self, value):
        self.__scan_area = value
        print(f'setting normal {self.__spim_pixels} and {self.__scan_area}')
        self.orsayscan.setImageArea(self.__scan_area[0], self.__scan_area[1], self.__scan_area[2], self.__scan_area[3],
                                    self.__scan_area[4], self.__scan_area[5])
        self.imagedata = numpy.zeros((self.__sizez * (self.__scan_area[0]), (self.__scan_area[1])), dtype=numpy.int16)
        self.imagedata_ptr = self.imagedata.ctypes.data_as(ctypes.c_void_p)

    @property
    def Spim_image_area(self):
        return self.__spim_pixels

    @Spim_image_area.setter
    def Spim_image_area(self, value):
        self.__spim_pixels = value
        print(f'setting spim {self.__spim_pixels} and {self.__scan_area}')
        self.spimscan.setImageArea(self.__spim_pixels[0], self.__spim_pixels[1], self.__scan_area[2], self.__scan_area[3],
                                    self.__scan_area[4], self.__scan_area[5])

    @property
    def probe_pos(self):
        return self.__probe_position

    @probe_pos.setter
    def probe_pos(self, value):
        self.__probe_position = value
        #This is a very strange behavior of geometry class. Very misleading. Value is a tuple like
        #(x=0.1, y=0.3) but value[0] gives y value while value[1] gives x value. You can check here
        #print(f'value is {value} and first is {value[0]}. Second is {value[1]}')
        #If you using this func, please call it with (y, x)
        px, py = round(self.__probe_position[1] * self.__scan_area[1]), round(
            self.__probe_position[0] * self.__scan_area[0])
        self.__probe_position_pixels = [px, py]
        self.orsayscan.SetProbeAt(px, py)

    @property
    def current_frame_parameters(self) -> scan_base.ScanFrameParameters:
        return self.__frame_parameters

    @property
    def is_scanning(self) -> bool:
        return self.__is_scanning

    @property
    def channel_count(self):
        return len(self.__channels)

    @property
    def channels_enabled(self) -> typing.Tuple[bool, ...]:
        return tuple(channel.enabled for channel in self.__channels)

    @property
    def set_spim(self):
        return self.__spim

    @set_spim.setter
    def set_spim(self, value):
        self.__spim = value

        #self.__is_scanning is false for spim. This allows us a better control of data flow. See first loop
        #condition in read_partial.

        """
        if value:
            if self.__is_scanning:
                self.orsayscan.stopImaging(True)
                self.__is_scanning = False
                logging.info('***SCAN***: Imaging was running. Turning it off...')

            if self.__instrument.spim_trigger_f==0:
                self.spimscan.setScanClock(2)
                logging.info(f'***SCAN***: EELS Spim')
            elif self.__instrument.spim_trigger_f==1:
                self.spimscan.setScanClock(4)
                logging.info(f'***SCAN***: Cathodoluminescence Spim')

            #self.imagedata = numpy.empty((self.__sizez * (self.__scan_area[0]), (self.__scan_area[1])), dtype=numpy.int16)
            #self.imagedata_ptr = self.imagedata.ctypes.data_as(ctypes.c_void_p)
            #print(f'set spim imagedatashape {self.imagedata.shape}')

            #self.spimscan.startSpim(0, 1)
        """
        if not value:
            logging.info('***SCAN***: Spim is done. Handling...')
            self.spimscan.stopImaging(True)
            self.__is_scanning = False

            pmts=[]
            for counter, value in enumerate(self.channels_enabled):
                if value: pmts.append(counter)
            self.__instrument.warn_Scan_instrument_spim_over(self.imagedata, self.__spim_pixels, pmts)

    @property
    def set_spim_pixels(self):
        return self.__spim_pixels

    @set_spim_pixels.setter
    def set_spim_pixels(self, value):
        pass
        #if value:
            #self.__spim_pixels = value
            #print(f'set spim pixels {self.__spim_pixels} and {self.__scan_area}')
            #self.spimscan.setImageArea(self.__spim_pixels[0], self.__spim_pixels[1], self.__scan_area[2], self.__scan_area[3], self.__scan_area[4],
            #                       self.__scan_area[5])
            #self.imagedata = numpy.empty((self.__sizez * (self.__scan_area[0]), (self.__scan_area[1])),
            #                             dtype=numpy.int16)
            #self.imagedata_ptr = self.imagedata.ctypes.data_as(ctypes.c_void_p)
            #print(f'set spim pixels imagedatashape {self.imagedata.shape}')

    def __data_locker(self, gene, datatype, sx, sy, sz):
        #sx[0] = self.__scan_area[0]
        #sy[0] = self.__scan_area[1]
        sz[0] = self.__sizez
        if gene == 1:
            sx[0] = self.Image_area[0]
            sy[0] = self.Image_area[1]
        else:
            sx[0] = self.Spim_image_area[0]
            sy[0] = self.Spim_image_area[1]

        datatype[0] = 2
        return self.imagedata_ptr.value

    #def __data_unlocker(self, gene, newdata):
    #    self.has_data_event.set()

    def __data_unlockerA(self, gene, newdata, imagenb, rect):
        if newdata:
            self.__frame_number = imagenb
            self.has_data_event.set()
            self.__line_number = rect[1] + rect[3]

    def show_configuration_dialog(self, api_broker) -> None:
        """Open settings dialog, if any."""
        api = api_broker.get_api(version="1", ui_version="1")
        document_controller = api.application.document_controllers[0]._document_controller
        myConfig = ConfigDialog(document_controller)

class ScanModule(scan_base.ScanModule):
    def __init__(self, instrument: ivg_inst.ivgInstrument) -> None:
        self.stem_controller_id = instrument.instrument_id
        self.device = Device(instrument)
        setattr(self.device, "priority", 20)
        scan_modes = (
            scan_base.ScanSettingsMode(_("Fast"), "fast", scan_base.ScanFrameParameters(pixel_size=(256, 256), pixel_time_us=1, fov_nm=4000)),
            scan_base.ScanSettingsMode(_("Slow"), "slow", scan_base.ScanFrameParameters(pixel_size=(512, 512), pixel_time_us=1, fov_nm=4000)),
            scan_base.ScanSettingsMode(_("Record"), "record", scan_base.ScanFrameParameters(pixel_size=(1024, 1024), pixel_time_us=1, fov_nm=4000))
        )
        self.settings = scan_base.ScanSettings(scan_modes, lambda d: scan_base.ScanFrameParameters(d), 0, 2)


def run(instrument: ivg_inst.ivgInstrument) -> None:
    Registry.register_component(ScanModule(instrument), {"scan_module"})

def stop() -> None:
    Registry.unregister_component(Registry.get_component("scan_module"), {"scan_module"})

#def run(instrument: ivg_inst.ivgInstrument):
#    scan_device = Device(instrument)
#    component_types = {"scan_device"}  # the set of component types that this component represents
#    Registry.register_component(scan_device, component_types)