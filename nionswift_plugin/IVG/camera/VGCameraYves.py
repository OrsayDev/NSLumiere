# standard libraries
import copy
import ctypes
import gettext
import numpy
import threading
import typing
import time
import json
from enum import Enum
import logging
from ctypes import c_uint64, c_int32

# local libraries
from nion.swift.model import HardwareSource
from nion.utils import Registry
from nion.utils import Event
from nion.instrumentation import camera_base
from nion.data import DataAndMetadata
from nion.instrumentation.camera_base import CameraFrameParameters

try:
    from ..aux_files import read_data
except ImportError:
    from ...aux_files import read_data

_ = gettext.gettext


# #Monkey patching camera_base because of the has_attr
# def test_update_spatial_calibrations(data_element, instrument_controller, camera, camera_category, data_shape, scaling_x, scaling_y):
#     if "spatial_calibrations" not in data_element:
#         if "spatial_calibrations" in data_element.get("hardware_source", dict()):
#             data_element["spatial_calibrations"] = data_element["hardware_source"]["spatial_calibrations"]
#         elif hasattr(camera, "calibration"):  # used in nionccd1010
#             data_element["spatial_calibrations"] = camera.calibration
#         elif instrument_controller:
#             if "calibration_controls" in data_element:
#                 calibration_controls = data_element["calibration_controls"]
#             elif hasattr(camera, "calibration_controls"):
#                 calibration_controls = camera.calibration_controls
#             else:
#                 calibration_controls = None
#             if calibration_controls is not None:
#                 x_calibration_dict = build_calibration_dict(instrument_controller, calibration_controls, "x", scaling_x, data_shape[0])
#                 y_calibration_dict = build_calibration_dict(instrument_controller, calibration_controls, "y", scaling_y, data_shape[1] if len(data_shape) > 1 else 0)
#                 z_calibration_dict = build_calibration_dict(instrument_controller, calibration_controls, "z", 1, data_shape[2] if len(data_shape) > 2 else 0)
#                 # leave this here for backwards compatibility until origin override is specified in NionCameraManager.py
#                 if camera_category.lower() == "ronchigram" and len(data_shape) == 2:
#                     y_calibration_dict["offset"] = -y_calibration_dict.get("scale", 1) * data_shape[0] * 0.5
#                     x_calibration_dict["offset"] = -x_calibration_dict.get("scale", 1) * data_shape[1] * 0.5
#                 if len(data_shape) == 1:
#                     data_element["spatial_calibrations"] = [x_calibration_dict]
#                 if len(data_shape) == 2:
#                     data_element["spatial_calibrations"] = [y_calibration_dict, x_calibration_dict]
#                 if len(data_shape) == 3:
#                     data_element["spatial_calibrations"] = [z_calibration_dict, y_calibration_dict, x_calibration_dict]
#
# camera_base.update_spatial_calibrations = test_update_spatial_calibrations


class Orsay_Data(Enum):
    s16 = 2
    s32 = 3
    uns16 = 6
    uns32 = 7
    float = 11
    real = 12


class CameraTask:
    def __init__(self, camera_device: "Camera", camera_frame_parameters, scan_shape: typing.Tuple[int, ...]):
        self.__camera_device = camera_device
        self.__camera_frame_parameters = camera_frame_parameters
        self.__scan_shape = scan_shape
        self.__scan_count = int(numpy.product(self.__scan_shape))
        self.__aborted = False
        self.__xdata: typing.Optional[DataAndMetadata.DataAndMetadata] = None
        self.__start = 0
        self.__start_time = 0
        self.__last_rows = 0
        self.__headers = False

    @property
    def xdata(self) -> typing.Optional[DataAndMetadata.DataAndMetadata]:
        return self.__xdata

    def grab_decoded_array(self):
        if self.__decode_array is not None:
            temp = self.__data.reshape(
                (self.__scan_shape[0] * self.__scan_shape[1], self.sizex))[self.__decode_array].reshape((self.__scan_shape[0], self.__scan_shape[1], self.sizex))
            self.__xdata = DataAndMetadata.new_data_and_metadata(
                temp, data_descriptor=self.__data_descriptor, metadata=self.__metadata)

    def prepare(self) -> None:
        # returns the full scan readout, including flyback pixels
        scan_size = int(numpy.product(self.__scan_shape))
        # now this works as before
        self.__last_rows = 0
        self.__headers = False
        self.__camera_device.frame_number = 0
        self.sizex, self.sizey = self.__camera_device.camera.getImageSize()
        settings = self.__camera_device.current_camera_settings.as_dict()
        # twoD = self.__camera_device.current_camera_settings.processing != "sum_project" \
        #               and not self.__camera_device.current_camera_settings.soft_binning
        self.__twoD = (self.sizey > 1) and not settings['soft_binning']
        self.__camera_device.current_camera_settings.processing = "None"
        datatype = numpy.float32
        # if "orsay" in settings.keys() and settings["orsay"]:
        # if hasattr(self.__camera_device, "isMedipix") and self.__camera_device.isMedipix:
        #     pixel_depth = self.__camera_device.pixeldepth
        #     if pixel_depth == 1:
        #         datatype = numpy.uint8
        #     elif pixel_depth == 6:
        #         if self.__twoD:
        #             datatype = numpy.uint8
        #         else:
        #             datatype = numpy.uint16
        #     elif pixel_depth == 12:
        #         if self.__twoD:
        #             datatype = numpy.uint16
        #         else:
        #             datatype = numpy.uint32
        if self.__twoD:
            self.sizez = scan_size
            reshape_array = (self.__scan_shape[0], self.__scan_shape[1], self.sizey, self.sizex)
            self.__camera_device.spimimagedata = numpy.zeros(reshape_array, dtype=datatype)
            camera_readout_shape = (self.sizey, self.sizex)
        else:
            self.sizey = scan_size
            self.sizez = 1
            reshape_array = (self.__scan_shape[0], self.__scan_shape[1], self.sizex)
            self.__camera_device.spimimagedata = numpy.zeros(reshape_array,
                                                             dtype=datatype)
            camera_readout_shape = (self.sizex,)
        print(f"Spim dimensions {self.sizex} {self.sizey} {self.sizez}")
        self.__data_descriptor = DataAndMetadata.DataDescriptor(False, len(self.__scan_shape),
                                                                len(camera_readout_shape))
        if settings['flipped']:
            self.__data = numpy.flip(self.__camera_device.spimimagedata, axis=2)
        else:
            self.__data = self.__camera_device.spimimagedata

        #This is for OpenScan only. It applies the mask so you can do any scan pattern without worrying
        scan_controller = Registry.get_component("stem_controller").scan_controller
        if scan_controller.scan_device.scan_device_id == "open_scan_device":
            self.__decode_array = scan_controller.scan_device.scan_engine.get_mask_array()
        else:
            self.__decode_array = numpy.zeros(0)

        #Adding metadata to the measurement
        self.__metadata = dict()
        self.__metadata['hardware_source'] = dict()
        self.__metadata['hardware_source']['decode_list'] = self.__decode_array.tolist()
        if self.__headers == False:
            if self.__camera_device.isMedipix:
                self.__metadata["hardware_source"]["merlin"] = dict()
                self.__metadata["hardware_source"]["merlin"]["acquisition"] = self.__camera_device.acquisition_header
                self.__metadata["hardware_source"]["merlin"]["image"] = self.__camera_device.image_header
        self.__xdata = DataAndMetadata.new_data_and_metadata(
            self.__data, data_descriptor=self.__data_descriptor, metadata=self.__metadata)

        #This call is for timepix3 only. Used to inform the correct scan size
        try:
            self.__camera_device.camera.set_scan_size(self.__scan_shape)
        except:
            logging.info(f'Could not set scan shape. This is normal in non-Timepix3 detectors.')
        self.__camera_device.camera.startSpim(scan_size, 1,
                                              self.__camera_device.current_camera_settings.exposure_ms / 1000,
                                              self.__twoD)

    def start(self) -> typing.Optional[DataAndMetadata.DataAndMetadata]:
        self.__camera_device.camera.resumeSpim(4)  # stop eof
        self.__start_time = time.time_ns()
        return self.__xdata

    def grab_partial(self, *, update_period: float = 1.0) -> typing.Tuple[bool, bool, int]:
        # updates the full scan readout data, returns a tuple of is complete, is canceled, and
        # the number of valid rows.
        elapsed_time = time.time_ns() - self.__start_time
        wait_time = update_period - elapsed_time * 1e-9
        if wait_time > 0.001:
            time.sleep(wait_time)
        self.__start_time = time.time_ns()
        if not self.__aborted:
            rows = self.__camera_device.frame_number // self.__scan_shape[1]
            is_complete = self.__camera_device.frame_number >= self.__scan_count
            # if self.__last_rows != rows or self.__last_rows == self.__scan_shape[0]:
            self.__last_rows = rows
            print(f"valid rows {rows},  frame number {self.__camera_device.frame_number}/{self.__scan_count}")
            return is_complete, False, rows
        return True, True, 0


class CameraDevice(camera_base.CameraDevice):

    def __init__(self, manufacturer, model, sn, simul, id, name, type):
        self.camera_id = id
        self.camera_type = type
        self.camera_name = name
        self.camera_model = model

        self.isKURO = model.find("KURO") >= 0
        self.isNewton = model.find("Newton") >= 0
        self.isProEM = model.find("ProEM") >= 0
        self.isMedipix = model.find("Merlin") >= 0
        self.isTimepix = model.find("CheeTah") >= 0

        if manufacturer == 4: #Timepix3
            from nionswift_plugin.IVG.tp3 import tp3func
            self.camera_callback = tp3func.SENDMYMESSAGEFUNC(self.sendMessageFactory())
            self.camera = tp3func.TimePix3(sn, simul, self.sendMessageFactory())
        elif manufacturer == 6: #To be quantum detector
            from nionswift_plugin.IVG.tp3 import tp3func
            self.camera_callback = tp3func.SENDMYMESSAGEFUNC(self.sendMessageFactory())
            self.camera = tp3func.TimePix3(sn, simul, self.sendMessageFactory())
        else:
            from nionswift_plugin.IVG.camera import orsaycamera
            self.camera = orsaycamera.orsayCamera(manufacturer, model, sn, simul)
        self.__config_dialog_handler = None
        self.__sensor_dimensions = self.camera.getCCDSize()
        self.__readout_area = 0, 0, *self.__sensor_dimensions
        self.__orsay_binning = self.camera.getBinning()
        self.__exposure_time = 0
        self.__is_vg = True
        self.sizex, self.sizey = self.camera.getImageSize()
        self.sizez = 1
        self._last_time = time.time()
        self.frame_parameter_changed_event = Event.Event()
        self.stop_acquitisition_event = Event.Event()
        self.current_event = Event.Event()

        # register data locker for focus acquisition
        if manufacturer != 4 and manufacturer != 6:
            self.fnlock = orsaycamera.DATALOCKFUNC(self.__data_locker)
            self.camera.registerDataLocker(self.fnlock)
            self.fnunlock = orsaycamera.DATAUNLOCKFUNC(self.__data_unlocker)
            self.camera.registerDataUnlocker(self.fnunlock)
        self.imagedata = None
        self.imagedata_ptr = None
        self.acquire_data = None
        self.has_data_event = threading.Event()

        # register data locker for SPIM acquisition
        if manufacturer != 4 and manufacturer != 6: #We dont do this for TPX3 and QD
            self.fnspimlock = orsaycamera.SPIMLOCKFUNC(self.__spim_data_locker)
            self.camera.registerSpimDataLocker(self.fnspimlock)

            #Some cameras use the old or the new unlocker!
            if self.isNewton:
                self.fnspimunlock = orsaycamera.SPIMUNLOCKFUNC(self.__spim_data_unlocker)
                self.camera.registerSpimDataUnlocker(self.fnspimunlock)
            else:
                self.fnspimunlockA = orsaycamera.SPIMUNLOCKFUNCA(self.__spim_data_unlockerA)
                self.camera.registerSpimDataUnlockerA(self.fnspimunlockA)

            self.fnspectrumlock = orsaycamera.SPECTLOCKFUNC(self.__spectrum_data_locker)
            self.camera.registerSpectrumDataLocker(self.fnspectrumlock)
            self.fnspectrumunlock = orsaycamera.SPECTUNLOCKFUNC(self.__spectrum_data_unlocker)
            self.camera.registerSpectrumDataUnlocker(self.fnspectrumunlock)
        self.spimimagedata = None
        self.spimimagedata_ptr = None
        self.has_spim_data_event = threading.Event()

        self.__cumul_on = False
        bx, by = self.camera.getBinning()
        port = self.camera.getCurrentPort()

        d = dict()
        d.update({
            "exposure_ms": 15,
            "h_binning": bx,
            "v_binning": by,
            "soft_binning": False,
            "acquisition_mode": "Focus",
            "synchro_mode": "Standalone",
            "spectra_count": 10,
            "multiplication": self.camera.getMultiplication()[0],
            "area": self.camera.getArea(),
            "port": port,
            "speed": self.camera.getCurrentSpeed(port),
            "gain": self.camera.getGain(port),
            "turbo_mode_enabled": self.camera.getTurboMode()[0],
            "video_threshold": self.camera.getVideoThreshold(),
            "fan_enabled": self.camera.getFan(),
            "processing": None,
            "flipped": False,
            "timeDelay": 0,
            "timeWidth": 0,
            "tp3mode": 0,
            "chips_config": 15,
            "gaps_mode": 0,
            "bias_V": 140,
        })

        self.current_camera_settings = CameraFrameParameters(d)
        self.__hardware_settings = self.current_camera_settings.as_dict()

        self.camera.setAccumulationNumber(self.current_camera_settings.as_dict()['spectra_count'])
        self.frame_number = 0

        self.__processing = None

        self.__acqon = False
        self.__x_pix_spim = 30
        self.__y_pix_spim = 30

        if manufacturer == 2:
            self.camera.setCCDOverscan(128, 0)

    @property
    def binning_values(self) -> typing.List[int]:
        return [1, 2, 5, 10, 20, 50, 200]

    def close(self):
        self.camera.stopSpim(True)
        # self.camera.close()

    def create_frame_parameters(self, d: dict) -> dict:
        return self.current_camera_settings

    def set_frame_parameters(self, frame_parameters) -> None:
        dict_frame_parameters = frame_parameters.as_dict()
        if self.__acqon:
            self.stop_live()

        if self.__hardware_settings['exposure_ms'] != dict_frame_parameters['exposure_ms']:
            self.__hardware_settings['exposure_ms'] = dict_frame_parameters['exposure_ms']
            self.camera.setExposureTime(dict_frame_parameters['exposure_ms'] / 1000.)
            if self.frame_parameter_changed_event is not None:
                self.frame_parameter_changed_event.fire("exposure_ms")

        if "acquisition_mode" in dict_frame_parameters:
            # if hasattr(frame_parameters, "acquisition_mode"):
            if self.frame_parameter_changed_event is not None:
                self.frame_parameter_changed_event.fire("acquisition_mode")

        if "soft_binning" in dict_frame_parameters:
            self.__hardware_settings['soft_binning'] = dict_frame_parameters['soft_binning']
            if self.__hardware_settings['acquisition_mode'] != dict_frame_parameters['acquisition_mode']:
                self.__hardware_settings['acquisition_mode'] = dict_frame_parameters['acquisition_mode']
            #print(f"***CAMERA***: acquisition mode[camera]: {self.__hardware_settings['acquisition_mode']}")
            self.__hardware_settings['spectra_count'] = dict_frame_parameters['spectra_count']

        if "port" in dict_frame_parameters:
            if self.__hardware_settings['port'] != dict_frame_parameters['port']:
                self.__hardware_settings['port'] = dict_frame_parameters['port']
                self.camera.setCurrentPort(dict_frame_parameters['port'])

        if "speed" in dict_frame_parameters:
            if self.__hardware_settings['speed'] != dict_frame_parameters['speed']:
                self.__hardware_settings['speed'] = dict_frame_parameters['speed']
                self.camera.setSpeed(self.__hardware_settings['port'], dict_frame_parameters['speed'])

        if "area" in dict_frame_parameters:
            # if hasattr(frame_parameters, "area"):
            if any(i != j for i, j in zip(self.__hardware_settings['area'], dict_frame_parameters['area'])):
                # if change area, put back binning to 1,1 temporarily in order to avoid conflicts, binnig will then be setup later
                v_binned = self.__hardware_settings['v_binning'] > 1
                self.__hardware_settings['h_binning'] = 1
                self.__hardware_settings['v_binning'] = 1
                self.camera.setBinning(self.__hardware_settings['h_binning'], self.__hardware_settings['v_binning'])
                if self.isMedipix:
                    self.__hardware_settings['area'] = dict_frame_parameters['area']
                    top = dict_frame_parameters['area'][0]
                    left = dict_frame_parameters['area'][1]
                    bottom = dict_frame_parameters['area'][2]
                    right = dict_frame_parameters['area'][3]
                    sx, sy = self.camera.getCCDSize()
                    szy = sy
                    sizey = sy - top
                    if sizey <= 4:
                        szy = 4
                    elif sizey <= 8:
                        szy = 8
                    elif sizey <= 16:
                        szy = 16
                    elif sizey <= 32:
                        szy = 32
                    elif sizey <= 64:
                        szy = 64
                    elif sizey <= 128:
                        szy = 128
                    self.__hardware_settings['area'] = [top - (sy - szy), left, bottom - (sy - szy), right, szy]
                    self.camera.setArea(self.__hardware_settings['area'])
                    if v_binned:
                        self.__hardware_settings['v_binning'] = int(bottom - top)
                        self.camera.setBinning(self.__hardware_settings['h_binning'], self.__hardware_settings['v_binning'])
                else:
                    self.__hardware_settings['area'] = dict_frame_parameters['area']
                    self.camera.setArea(self.__hardware_settings['area'])

        if ("h_binning" in dict_frame_parameters) and ("v_binning" in dict_frame_parameters):
            # if hasattr(frame_parameters, "h_binning") and hasattr(frame_parameters, "v_binning"):
            if (self.__hardware_settings['h_binning'] != dict_frame_parameters['h_binning']) \
                    or (self.__hardware_settings['v_binning'] != dict_frame_parameters['v_binning']):
                self.camera.setBinning(dict_frame_parameters['h_binning'], dict_frame_parameters['v_binning'])
                self.__hardware_settings['h_binning'], self.__hardware_settings['v_binning'] = self.camera.getBinning()

        if "gain" in dict_frame_parameters:
            if self.__hardware_settings['gain'] != dict_frame_parameters['gain']:
                 self.__hardware_settings['gain'] = dict_frame_parameters['gain']
                 self.camera.setGain(self.__hardware_settings['gain'])

        if (not self.isKURO) and "multiplication" in dict_frame_parameters:
            if self.__hardware_settings['multiplication'] != dict_frame_parameters['multiplication']:
                self.__hardware_settings['multiplication'] = dict_frame_parameters['multiplication']
                self.camera.setMultiplication(self.__hardware_settings['multiplication'])

        if "spectra_count" in dict_frame_parameters:
            self.__hardware_settings['spectra_count'] = dict_frame_parameters['spectra_count']
            self.camera.setAccumulationNumber(self.__hardware_settings['spectra_count'])

        if "video_threshold" in dict_frame_parameters:
            self.__hardware_settings['video_threshold'] = dict_frame_parameters['video_threshold']
            self.camera.setVideoThreshold(self.__hardware_settings['video_threshold'])

        if "fan_enabled" in dict_frame_parameters:
            self.__hardware_settings['fan_enabled'] = dict_frame_parameters['fan_enabled']
            self.camera.setFan(self.__hardware_settings['fan_enabled'])

        if "processing" in dict_frame_parameters:
            self.__hardware_settings['processing'] = dict_frame_parameters['processing']

        if "flipped" in dict_frame_parameters:
            if self.__hardware_settings['flipped'] != dict_frame_parameters['flipped']:
                self.__hardware_settings['flipped'] = dict_frame_parameters['flipped']


        #Timepix3 camera values
        if self.isTimepix and "timeDelay" in dict_frame_parameters:
            self.__hardware_settings['timeDelay'] = dict_frame_parameters['timeDelay']
            self.camera.setDelayTime(dict_frame_parameters['timeDelay'])

        if self.isTimepix and "timeWidth" in dict_frame_parameters:
            self.__hardware_settings['timeWidth'] = dict_frame_parameters['timeWidth']
            self.camera.setWidthTime(dict_frame_parameters['timeWidth'])

        if self.isTimepix and "tp3mode" in dict_frame_parameters:
            self.__hardware_settings['tp3mode'] = dict_frame_parameters['tp3mode']
            self.camera.setTp3Mode(dict_frame_parameters['tp3mode'])

        if self.isTimepix and "bias_V" in dict_frame_parameters:
            if self.__hardware_settings['bias_V'] != dict_frame_parameters['bias_V']:
                self.__hardware_settings['bias_V'] = dict_frame_parameters['bias_V']
                self.camera.setTp3BiasVoltage(dict_frame_parameters['bias_V'])

        #Medipix3 camera values
        if self.isMedipix and "chips_config" in dict_frame_parameters:
            if self.__hardware_settings['chips_config'] != dict_frame_parameters['chips_config']:
                self.__hardware_settings['chips_config'] = dict_frame_parameters['chips_config']
                self.camera.chips_config = self.__hardware_settings['chips_config']

        if (self.isMedipix or self.isTimepix) and "gaps_mode" in dict_frame_parameters:
            if self.__hardware_settings['gaps_mode'] != dict_frame_parameters['gaps_mode']:
                self.__hardware_settings['gaps_mode'] = dict_frame_parameters['gaps_mode']
                self.camera.gaps_mode = self.__hardware_settings['gaps_mode']

    def __numpy_to_orsay_type(self, array: numpy.array):
        orsay_type = Orsay_Data.float
        if array.dtype == numpy.double:
            orsay_type = Orsay_Data.real
        if array.dtype == numpy.int16:
            orsay_type = Orsay_Data.s16
        if array.dtype == numpy.int32:
            orsay_type = Orsay_Data.s32
        if array.dtype == numpy.uint16:
            orsay_type = Orsay_Data.uns16
        if array.dtype == numpy.uint32:
            orsay_type = Orsay_Data.uns32
        return orsay_type.value

    def __connection_listener(self, changed: bool, message: bool):
        if message:
            print("Message connection changed: " + changed)
        else:
            print("Data connection changed: " + changed)

    def __data_locker(self, gene, data_type, sx, sy, sz):
        sx[0] = self.sizex
        sy[0] = self.sizey
        sz[0] = 1
        data_type[0] = self.__numpy_to_orsay_type(self.imagedata)
        if self.imagedata_ptr is None:
            return None
        else:
            return self.imagedata_ptr.value

    def __data_unlocker(self, gene, new_data):
        self.frame_number += 1
        status = self.camera.getCCDStatus()
        if new_data:
            t = time.time()
            if t - self._last_time > 0.1:
                self._last_time = t
        self.has_data_event.set()
        if status["mode"] == "Cumul":
            self.frame_number = status["accumulation_count"]
        if self.__cumul_on and status["mode"] == "idle":
            hardware_source = HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id(
                self.camera_id)
            hardware_source.stop_playing()

    def __spim_data_locker(self, gene, data_type, sx, sy, sz):
        sx[0] = self.__x_pix_spim
        sy[0] = self.__y_pix_spim
        sz[0] = self.sizez
        #data_type >= 100 force spectrum data on first axis.
        data_type[0] = 100 + self.__numpy_to_orsay_type(self.spimimagedata)
        return self.spimimagedata_ptr.value

    def __spim_data_unlocker(self, gene :int, new_data : bool, running : bool):
        status = self.camera.getCCDStatus()
        # if status["mode"] == "Spectrum imaging":
        self.frame_number = int(status["current spectrum"])
        if "Chrono" in status["mode"]:
            if new_data:
                self.has_data_event.set()
        else:
            if not running:
                # just stopped, send last data anyway.
                self.has_spim_data_event.set()
                print(f"spim done => frames {self.frame_number}")
        if not running:
            hardware_source = HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id(
                self.camera_id)
            hardware_source.stop_playing()

    def __spim_data_unlockerA(self, gene : int, new_data : bool, current_spectrum: c_uint64, current_spim : c_int32, running : bool):
        self.frame_number = current_spectrum
        status = self.camera.getCCDStatus()
        if "Chrono" in self.current_camera_settings.as_dict()['acquisition_mode']:
            if new_data:
                self.has_data_event.set()
        else:
            if not running:
                # just stopped, send last data anyway.
                self.has_spim_data_event.set()
                print(f"spim done => frames {self.frame_number}")
        if not running:
            hardware_source = HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id(
                self.camera_id)
            hardware_source.stop_playing()

    def __spectrum_data_locker(self, gene, data_type, sx) -> None:
        if self.__acqon and (self.current_camera_settings.exposure_ms >= 10):
            sx[0] = self.sizex
            data_type[0] = self.__numpy_to_orsay_type(self.imagedata)
            return self.imagedata_ptr.value
        else:
            return None

    def __spectrum_data_unlocker(self, gene, newdata):
        if "Chrono" in self.current_camera_settings.as_dict()['acquisition_mode']:
            self.has_data_event.set()

    @property
    def sensor_dimensions(self) -> (int, int):
        return self.__sensor_dimensions

    @property
    def readout_area(self) -> (int, int, int, int):
        return self.__readout_area

    @readout_area.setter
    def readout_area(self, readout_area_TLBR: (int, int, int, int)) -> None:
        self.__readout_area = readout_area_TLBR

    @property
    def flip(self):
        return False

    @flip.setter
    def flip(self, do_flip):
        pass

    def start_live(self) -> None:
        self.frame_number = 0
        self.__cumul_on = False
        self.sizex, self.sizey = self.camera.getImageSize()
        if self.current_camera_settings.as_dict()['soft_binning']:
            self.sizey = 1

        if "Chrono" in self.current_camera_settings.as_dict()['acquisition_mode']:
            if self.current_camera_settings.as_dict()['acquisition_mode'] == '2D-Chrono':
                self.sizez = self.current_camera_settings.spectra_count
                if not self.isTimepix: self.spimimagedata = numpy.zeros((self.sizez, self.sizey, self.sizex), dtype=numpy.float32)
            else:
                self.sizey = self.current_camera_settings.as_dict()['spectra_count']
                self.sizez = 1
                if not self.isTimepix: self.spimimagedata = numpy.zeros((self.sizey, self.sizex), dtype=numpy.float32)
            if not self.isTimepix: self.spimimagedata_ptr = self.spimimagedata.ctypes.data_as(ctypes.c_void_p)
            self.camera.stopFocus()
            if self.isTimepix:
                sb = "1d" if self.current_camera_settings.as_dict()['soft_binning'] else "2d"
                if self.current_camera_settings.as_dict()['acquisition_mode'] == "1D-Chrono-Live":
                    acqmode = 1  # Chrono Live
                else:
                    acqmode = 0  # Normal Chrono
                self.__acqon = self.camera.startChrono(self.current_camera_settings.as_dict()['exposure_ms'] / 1000, sb,
                                                       acqmode)
                self.camera._TimePix3__isReady.wait(5.0)
                self.spimimagedata = self.camera.create_specimage()
            else:
                self.camera.startSpim(self.current_camera_settings.as_dict()['spectra_count'], 1,
                                      self.current_camera_settings.as_dict()['exposure_ms'] / 1000.,
                                      self.current_camera_settings.as_dict()['acquisition_mode'] == "2D-Chrono")
            self.camera.resumeSpim(4)
            if self.current_camera_settings.as_dict()['acquisition_mode'] == "1D-Chrono-Live":
                self.camera.setSpimMode(1)


        elif "Event Hyperspec" in self.current_camera_settings.as_dict()['acquisition_mode']:
            self.camera.stopFocus()
            self.camera.StartSpimFromScan()
            self.camera._TimePix3__isReady.wait(5.0)
            assert self.isTimepix == True
            self.spimimagedata = self.camera.create_spimimage()

        else:
            self.sizez = 1
            acqmode = 0
            sb = "1d" if self.current_camera_settings.as_dict()['soft_binning'] else "2d"
            if self.current_camera_settings.as_dict()['acquisition_mode'] == "Cumul":
                acqmode = 1
                self.__cumul_on = True
            if not self.isTimepix:
                self.imagedata = numpy.zeros((self.sizey, self.sizex), dtype=numpy.float32)
                self.imagedata_ptr = self.imagedata.ctypes.data_as(ctypes.c_void_p)
            self.__acqon = self.camera.startFocus(self.current_camera_settings.as_dict()['exposure_ms'] / 1000, sb,
                                                  acqmode)
            if self.isTimepix:
                self.camera._TimePix3__isReady.wait(5.0)
                self.imagedata = self.camera.create_specimage()

        self._last_time = time.time()

    def stop_live(self) -> None:
        if self.__acqon:
            self.camera.stopFocus()
            self.__acqon = False
        if "Chrono" in self.current_camera_settings.as_dict()['acquisition_mode'] \
                or ("Focus" in self.current_camera_settings.as_dict()['acquisition_mode']) \
                or "Event Hyperspec" in self.current_camera_settings.as_dict()['acquisition_mode']:
            self.camera.stopSpim(True)
            self.__acqon = False
            logging.info('***CAMERA***: Spim stopped. Handling...')

    def acquire_image(self) -> dict:
        self.has_data_event.wait(1)
        self.has_data_event.clear()
        acquisition_mode = self.current_camera_settings.as_dict()['acquisition_mode']

        if "Chrono" in acquisition_mode:
            self.acquire_data = self.spimimagedata
            if "2D" in acquisition_mode:
                collection_dimensions = 1
                datum_dimensions = 2
            else:
                collection_dimensions = 0
                datum_dimensions = 2

        elif "Event Hyperspec" in acquisition_mode:
            self.acquire_data = self.spimimagedata
            collection_dimensions = 2
            datum_dimensions = 1

        else:
            self.acquire_data = self.imagedata
            if self.acquire_data.shape[0] == 1: #fully binned
                self.acquire_data = self.acquire_data.reshape(self.acquire_data.shape[1])
                datum_dimensions = 1
                collection_dimensions = 1
                if self.current_camera_settings.as_dict()['flipped']:
                    self.acquire_data = numpy.flip(self.acquire_data, axis=0)
            else:
                datum_dimensions = 2
                collection_dimensions = 0
                if self.current_camera_settings.as_dict()['flipped']:
                    self.acquire_data = numpy.flip(self.acquire_data, axis=1)


        properties = dict()
        properties["frame_number"] = self.frame_number
        properties["acquisition_mode"] = acquisition_mode
        properties["frame_parameters"] = dict(self.current_camera_settings.as_dict())
        calibration_controls = copy.deepcopy(self.calibration_controls)

        if self.isMedipix:
            properties["merlin"] = dict()
            properties["merlin"]["acquisition"] = self.acquisition_header
            properties["merlin"]["acquisition"] = self.image_header


        return {"data": self.acquire_data, "collection_dimension_count": collection_dimensions,
                "datum_dimension_count": datum_dimensions, "calibration_controls": calibration_controls,
                "properties": properties}

    def acquire_synchronized_begin(self, camera_frame_parameters: camera_base.CameraFrameParameters,
                                   scan_shape: DataAndMetadata.ShapeType,
                                   **kwargs: typing.Any) -> camera_base.PartialData:

        self.__camera_task = CameraTask(self, camera_frame_parameters, scan_shape)
        self.__camera_task.prepare()
        self.__x_pix_spim = scan_shape[1]
        self.__y_pix_spim = scan_shape[0]
        self.sizez = self.spimimagedata.shape[2]
        self.spimimagedata_ptr = self.spimimagedata.ctypes.data_as(ctypes.c_void_p)
        self.__camera_task.start()
        return camera_base.PartialData(self.__camera_task.xdata, False, False, 0)

    def acquire_synchronized_continue(self, *, update_period: float = 1.0,
                                      **kwargs: typing.Any) -> camera_base.PartialData:
        # assert self.__camera_task
        is_complete, is_canceled, valid_count = self.__camera_task.grab_partial(update_period=3.0)
        return camera_base.PartialData(self.__camera_task.xdata, is_complete, is_canceled, valid_count)

    def acquire_synchronized_end(self, **kwargs: typing.Any) -> None:
        self.camera.stopSpim(True)
        self.__camera_task = None

    def acquire_synchronized_cancel(self) -> None:
        self.__cancel_sequence_event.set()

    @property
    def _is_acquire_synchronized_running(self) -> bool:
        return self.__camera_task is not None

    @property
    def calibration_controls(self) -> dict:
        if self.camera_type == "eels":
            return {
                "x_scale_control": "EELS_TV_eVperpixel",
                "x_offset_control": "KURO_EELS_eVOffset",
                "x_units_value": "eV",
                "y_scale_control": self.camera_type + "_y_scale",
                "y_offset_control": self.camera_type + "_y_offset",
                "y_units_value": "",
                "intensity_units_value": "counts",
                "counts_per_electron_value": 1
            }
        elif self.camera_type == "eire":
            return {
                "x_scale_control": "Princeton_CL_nmperpixel",
                "x_offset_control": "Princeton_CL_nmOffset",
                "x_units_value": "nm",
                "y_scale_control": "Princeton_CL_radsperpixel",
                "y_units_value": "rad",
                "intensity_units_value": "counts",
                "counts_per_electron_value": "Princeton_CL_CountsPerElectron"
            }
        else:
            return {
                "x_scale_control": self.camera_type + "_x_scale",
                "x_offset_control": self.camera_type + "_x_offset",
                "x_units_value": "eV" if self.camera_type == 'eels' else "nm",
                "y_scale_control": self.camera_type + "_y_scale",
                "y_offset_control": self.camera_type + "_y_offset",
                "y_units_value": "",
                "intensity_units_value": "counts",
                "counts_per_electron_value": 1
            }



    @property
    def processing(self) -> typing.Optional[str]:
        return self.__processing

    @processing.setter
    def processing(self, value: str) -> None:
        self.__processing = value

    def get_expected_dimensions(self, binning: int) -> (int, int):
        sx, sy = self.camera.getImageSize()
        return sy, sx

    def start_monitor(self) -> None:
        pass

    #def acquisition_header(self):
    #    return json.loa

    @property
    def fan_enabled(self) -> bool:
        return self.camera.getFan()

    @fan_enabled.setter
    def fan_enabled(self, value: bool) -> None:
        self.camera.setFan(bool(value))

    @property
    def pixeldepth(self) -> int:
        return self.camera.pixeldepth

    @property
    def acquisition_header(self) -> dict:
        return json.loads(self.camera.acquisition_header)

    @property
    def image_header(self) -> dict:
        return json.loads(self.camera.image_header)

    def isCameraAcquiring(self):
        return self.__acqon

    def __getTurboMode(self):
        value, hs, vs = self.camera.getTurboMode()
        print(f"turbo mode : {value}")
        return value

    @property
    def readoutTime(self) -> float:
        return self.camera.getReadoutTime()

    def get_acquire_sequence_metrics(self, camera_frame_parameters: typing.Dict) -> typing.Dict:
        camera_frame_parameters = camera_frame_parameters.as_dict()
        acquisition_frame_count = camera_frame_parameters.get("acquisition_frame_count")
        storage_frame_count = camera_frame_parameters.get("storage_frame_count")
        frame_time = self.current_camera_settings.exposure_ms / 1000 + self.camera.getReadoutTime()
        acquisition_time = frame_time * acquisition_frame_count
        if camera_frame_parameters.get("processing") == "sum_project":
            acquisition_memory = self.camera.getImageSize()[0] * 4 * acquisition_frame_count
            storage_memory = self.camera.getImageSize()[0] * 4 * storage_frame_count
        else:
            acquisition_memory = self.camera.getImageSize()[0] * self.camera.getImageSize()[
                1] * 4 * acquisition_frame_count
            storage_memory = self.camera.getImageSize()[0] * self.camera.getImageSize()[1] * 4 * storage_frame_count
        return {"acquisition_time": acquisition_time, "acquisition_memory": acquisition_memory,
                "storage_memory": storage_memory}

    def sendMessageFactory(self):
        """
        Notes
        -----

        SendMessageFactory is a standard callback function encountered in several other instruments. It allows main file
        to receive replies from the instrument (or device class). These callbacks are normally done by the standard
        unlock function. They set events that tell acquisition thread new data is available.

        As TimePix3 instantiation is done in python, those callback functions are explicitely defined
        here. This means that sendMessageFactory are only supossed to be used by TimePix3 by now. If other cameras
        are instantiated in python in the future, they could use exactly same scheme. Note that all Marcel
        implementations, like stopFocus, startSpim, etc, are defined in tp3func.

        The callback are basically events that tell acquire_image that a new data is available for displaying. In my case,
        message equals to 01 is equivalent to Marcel's data locker, while message equals to 02 is equivalent to spim data
        locker. Data locker (message==1) gets data from a LIFOQueue, which is a tuple in which first element is the frame
        properties and second is the data (in bytes). You can see what is available in dict 'prop' checking either
        serval manual or tp3func. create_image_from_bytes simply convert my bytes to a int8 array. A soft binning attribute
        is defined in tp3 so the idea is that image always come in the right way.

        For message==2, it is exactly the same. Difference is simply dimensionality (datum and collection dimensions) and,
        if array is complete, i double the size in order to always show more data. A personal choice to never limit data
        arrival.
        """

        def sendMessage(message):
            self.frame_number = self.camera.get_frame()
            if message == 1:
                self.current_event.fire(format(self.camera.get_current(self.imagedata, self.frame_number), ".5f"))
                self.has_data_event.set()
            elif message == 2:
                self.spimimagedata[:] = self.camera.create_spimimage_frame()
                self.has_spim_data_event.set()
            elif message == 3:
                self.has_data_event.set()
        return sendMessage


class CameraSettings():
    def __init__(self, camera_device: CameraDevice):
        # these events must be defined
        self.current_frame_parameters_changed_event = Event.Event()
        self.record_frame_parameters_changed_event = Event.Event()
        self.profile_changed_event = Event.Event()
        self.frame_parameters_changed_event = Event.Event()
        self.settings_changed_event = Event.Event()
        self.__camera_device = camera_device

        # the list of possible modes should be defined here
        if camera_device.isTimepix:
            self.modes = ["Focus", "Cumul", "1D-Chrono", "1D-Chrono-Live", "2D-Chrono", "Event Hyperspec"]
        else:
            self.modes = ["Focus", "Cumul", "1D-Chrono", "1D-Chrono-Live", "2D-Chrono"]
        self.synchro_modes = ["Standalone", "Master", "Slave"]
        self.settings_id = camera_device.camera_id

    def close(self):
        pass

    def initialize(self, **kwargs):
        pass

    def apply_settings(self, settings_dict: typing.Dict) -> None:
        """Initialize the settings with the settings_dict."""
        if isinstance(settings_dict, dict):
            self.__frame_parameters = CameraFrameParameters(settings_dict)
            self.__record_parameters = copy.deepcopy(settings_dict)
            self.__camera_device.current_camera_settings = self.__frame_parameters
            self.settings_changed_event.fire(self.__frame_parameters)

    def __save_settings(self) -> typing.Dict:
        return self.__frame_parameters.as_dict()

    def get_frame_parameters_from_dict(self, d):
        return CameraFrameParameters(d)

    def set_current_frame_parameters(self, frame_parameters: CameraFrameParameters) -> None:
        self.__camera_device.current_camera_settings = frame_parameters
        self.settings_changed_event.fire(frame_parameters.as_dict())
        self.current_frame_parameters_changed_event.fire(frame_parameters)
        self.record_frame_parameters_changed_event.fire(frame_parameters)

    def get_current_frame_parameters(self) -> CameraFrameParameters:
        return self.__camera_device.current_camera_settings

    def set_record_frame_parameters(self, frame_parameters: CameraFrameParameters) -> None:
        self.set_current_frame_parameters(frame_parameters)

    def get_record_frame_parameters(self) -> CameraFrameParameters:
        return self.get_current_frame_parameters()

    def set_frame_parameters(self, profile_index: int, frame_parameters) -> None:
        self.set_current_frame_parameters(frame_parameters)

    def get_frame_parameters(self, profile_index: int):
        return self.get_current_frame_parameters()

    def set_selected_profile_index(self, profile_index: int) -> None:
        pass

    @property
    def selected_profile_index(self) -> int:
        return 0

    def get_mode(self) -> str:
        return str()

    def set_mode(self, mode: str) -> None:
        pass

    def open_configuration_interface(self, api_broker) -> None:
        pass

    def open_monitor(self) -> None:
        pass


class CameraModule:

    def __init__(self, stem_controller_id: str, camera_device: CameraDevice, camera_settings: CameraSettings):
        self.stem_controller_id = stem_controller_id
        self.camera_device = camera_device
        self.camera_settings = camera_settings
        self.camera_panel_type = "orsay_camera_panel"


def periodic_logger():
    messages = list()
    data_elements = list()
    return messages, data_elements

def run():
    set_file = read_data.FileManager('Orsay_cameras_list')

    for camera in set_file.settings:
        try:
            sn = ""
            if camera["manufacturer"] == 1:
                manufacturer = "Roperscientific"
            elif camera["manufacturer"] == 2:
                manufacturer = "Andor"
            elif camera["manufacturer"] == 3:
                manufacturer = "QuantumDetectors"
                sn = camera["ip_address"]
            elif camera["manufacturer"] == 4:
                manufacturer = "AmsterdamScientificInstruments"
                sn = camera["ip_address"]
            elif camera["manufacturer"] == 6:
                manufacturer = "QuantumDetectorsPY"
                sn = camera["ip_address"]
            model = camera["model"]
            if (camera["manufacturer"] == 2) and camera["simulation"]:
                logging.info(f"***CAMERA***: No simulation for {manufacturer} camera.")
            else:
                camera_device = CameraDevice(camera["manufacturer"], camera["model"], sn, camera["simulation"],
                                             camera["id"], camera["name"], camera["type"])

                camera_settings = CameraSettings(camera_device)
                Registry.register_component(CameraModule("orsay_controller", camera_device, camera_settings),
                                            {"camera_module"})
            set_file.save_locally()
        except Exception as e:
            logging.info(f"***CAMERA***: Failed to start camera: {manufacturer}.  model: {model}. Exception: {repr(e)}")
