# standard libraries
import copy, math, gettext, numpy, typing, time, threading, logging, os, sys, json, traceback

# local libraries
from nion.utils import Registry
from nion.utils import Geometry
from nion.utils import Event
from nion.instrumentation import scan_base, stem_controller
from nion.instrumentation import HardwareSource

from nionswift_plugin.IVG.scan.OScanCesysDialog import ConfigDialog
from FPGAControl import FPGAConfig
from ...aux_files import read_data

from .OScanCesysDialog import KERNEL_LIST, ACQUISITION_WINDOW, SCAN_MODES, IMAGE_VIEW_MODES, ADC_READOUT_MODES

_ = gettext.gettext

set_file = read_data.FileManager('global_settings')
OPEN_SCAN_IS_VG = set_file.settings["OrsayInstrument"]["open_scan"]["IS_VG"]
OPEN_SCAN_EFM03 = set_file.settings["OrsayInstrument"]["open_scan"]["EFM03"]
OPEN_SCAN_BITSTREAM = set_file.settings["OrsayInstrument"]["open_scan"]["BITSTREAM_FILE"]
DEBUG = False
TIMEOUT_IS_SYNC = 2.0

def getlibname():
    if sys.platform.startswith('win'):
        libname = os.path.join(os.path.dirname(__file__), "../../aux_files/DLLs/")
    else:
        libname = os.path.join(os.path.dirname(__file__), "../../aux_files/DLLs/")
    return libname


class ArgumentController:
    """
    ArgumentController stores all the arguments of the ScanDevice into a dictionary. Useful for persistent settings and control.
    """
    def __init__(self, filename: str):
        self.__settings_manager = read_data.FileManager(filename)
        self.argument_controller = self.__settings_manager.settings

    def get(self, keyname: str, value=None):
        #If the keyname does not exists and the value is not None, we set the value to populate the dictionary
        if keyname not in self.argument_controller and value is not None:
            self.set(keyname, value)
        return self.argument_controller.get(keyname, value)

    def set(self, keyname: str, value):
        self.argument_controller[keyname] = value
        self._write_to_json()

    def keys(self):
        return self.argument_controller.keys()

    def update(self, **kwargs):
        self.argument_controller.update(**kwargs)
        self._write_to_json()

    def _write_to_json(self):
        self.__settings_manager.save_locally()


class ScanEngine:
    def __init__(self):
        self.property_changed_event = Event.Event()
        try:
            self.debug_io = None
            io_system = FPGAConfig.CesysDevice(getlibname().encode(),
                                               OPEN_SCAN_BITSTREAM)
            self.device = FPGAConfig.ScanDevice(io_system, 128, 128, 100, 0)
        except UnboundLocalError:
            self.debug_io = FPGAConfig.DebugClass()
            self.device = FPGAConfig.ScanDevice(self.debug_io, 128, 128, 100, 0)
            logging.warning(f'Could not found CESYS system connected. Entering in debug mode.')
        except:
            self.debug_io = FPGAConfig.DebugClass()
            self.device = FPGAConfig.ScanDevice(self.debug_io, 128, 128, 100, 0)
            logging.warning(f'Could not found the library libudk3-1.5.1.so. You should probably use '
                            f'export LD_LIBRARY_PATH=' + getlibname())

        # Settings
        self.__presetting = 0
        self.__last_frame_parameters: scan_base.ScanFrameParameters = None
        self.__last_frame_parameters_time = time.time()
        self.__last_probe_position = (0.5, 0.5)
        self.argument_controller = ArgumentController('opscan_persistent_data_' + str(self.__presetting))

        # Initializating values from the json file, if exists
        for keys in self.argument_controller.keys():
            try:
                setattr(self, keys, getattr(self, keys))
            except AttributeError:
                logging.info(f"***OScan***: Could not set key {keys} in the scan engine.")

        # Set of settings that gets overwritten. It makes sure the initial state is user-friendly
        self.adc_acquisition_mode = 5 #Taped FIR
        self.imagedisplay = 1 #Normal display
        self.rastering_mode = 0 #Normal mode
        self.image_cumul = 1 #No accumulation
        self.flyback_us = 30 #10 us flyback
        self.enable_x_delay = False #No delay on X coil
        self.enable_y_delay = False #No delay on Y coil


    def receive_total_frame(self, channel: int):
        image = self.device.get_image(channel, imageType=IMAGE_VIEW_MODES[self.imagedisplay],
                                      low_pass_size=self.imagedisplay_filter_intensity)
        return image

    def update_metadata_to_dict(self, metadata: dict):
        """"
        Being sure that the frame parameters are updated during frame acquisition
        """
        metadata.update(self.argument_controller.argument_controller)

    def get_ordered_array(self):
        """
        Get the last ordered array parsed as a list to the engine
        """
        return self.device.get_ordered_array()

    def get_mask_array(self):
        """
        Get the last masked (decoded) array parsed as a list to the engine
        """
        return self.device.get_mask_array()

    def set_frame_parameters(self, frame_parameters: scan_base.ScanFrameParameters):
        """
        Sets the frame parameters of the scan. The frame_parameters must be different in order to this to be taken into account.
        """
        is_synchronized_scan = frame_parameters.get_parameter("external_clock_mode", 0)
        (y, x) = frame_parameters.as_dict()['pixel_size']
        pixel_time = frame_parameters.as_dict()['pixel_time_us']
        fov_nm = frame_parameters.as_dict()['fov_nm']
        rotation_rad = frame_parameters.as_dict().get('rotation_rad', 0.0)
        subscan_fractional_size = frame_parameters.as_dict().get('subscan_fractional_size')
        subscan_fractional_center = frame_parameters.as_dict().get('subscan_fractional_center')
        subscan_pixel_size = frame_parameters.as_dict().get('subscan_pixel_size')

        # Setting the field of view. This does not need to change the list
        if self.__last_frame_parameters is None or fov_nm != self.__last_frame_parameters.as_dict()['fov_nm']:
            self.set_field_of_view(fov_nm)
            if self.__last_frame_parameters is not None: self.__last_frame_parameters.set_parameter('fov_nm', fov_nm)

        # Setting the values in the frame parameters to be compared in the next step
        for (key, value) in self.argument_controller.argument_controller.items():
            frame_parameters.set_parameter(key, value)

        if self.__last_frame_parameters is None or frame_parameters.as_dict() != self.__last_frame_parameters.as_dict():
            self.device.change_scan_parameters(x, y, pixel_time, self.flyback_us, is_synchronized_scan,
                                               SCAN_MODES[self.rastering_mode],
                                               rotation_rad=rotation_rad,
                                               lissajous_nx=self.lissajous_nx,
                                               lissajous_ratio=self.lissajous_ratio,
                                               lissajous_phase=self.lissajous_phase,
                                               subimages=self.mini_scan,
                                               adc_acquisition_mode=self.adc_acquisition_mode,
                                               adc_acquisition_mode_name=ADC_READOUT_MODES[self.adc_acquisition_mode],
                                               kernelMode=KERNEL_LIST[self.kernel_mode],
                                               givenPixel=self.given_pixel,
                                               dutyCycle=self.duty_cycle,
                                               acquisitionCutoff=self.acquisition_cutoff,
                                               acquisitionWindow=self.acquisition_window,
                                               subscan_fractional_size=subscan_fractional_size,
                                               subscan_fractional_center=subscan_fractional_center,
                                               subscan_pixel_size=subscan_pixel_size
                                               )

        self.__last_frame_parameters_time = time.time()
        self.__last_frame_parameters = frame_parameters

    def _update_frame_parameter(self):
        """
        Try to updates the frame_parameter. The self.__last_frame_parameter is copied but its updated with all the values on ArgumentController.
        If any of them change, they trigger a scan frame parameter change
        """
        updated_frame_parameter = copy.deepcopy(self.__last_frame_parameters)
        if self.__last_frame_parameters is not None:
            self.set_frame_parameters(updated_frame_parameter)

    def set_field_of_view(self, fov: float):
        """
        Sets the field of the view of the image
        """
        self.device.change_magnification_values(fov)
        superscan = HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id("superscan")
        if superscan is not None:
            ss_fp = superscan.get_current_frame_parameters()
            ss_fp.set_parameter("fov_nm", fov)
            superscan.set_current_frame_parameters(ss_fp)

    def set_probe_position(self, x, y):
        """
        Sets the probe position. This gots called multiple times so the condition of the last_frame_parameter_time.
        """
        temp_time = time.time()
        if temp_time - self.__last_frame_parameters_time > 0.1:
            self.__last_probe_position = (x, y)
            self.device.set_probe_position(x, y)
            #This will force the next frame to be taken place
            self.__last_frame_parameters = None

    def get_mask_array(self):
        return self.device.get_mask_array()

    @property
    def pre_settings(self):
        return self.__presetting

    @pre_settings.setter
    def pre_settings(self, value):
        """
        This changes the argument controller file in order to have different presettings. This is difficult to control
        properly and very efficiently.

        If the key does not exists in the new presetting, a copy from the previous values are created. If the key exists
        and the value differs, we set the property in order to take place. Some of these properties need the
        frame_parameters, while others do not. This risks to call set_frame_parameters more than once.

        The self.__last_frame_parameters = None means that for these internal functions wont be called so you need to
        restart the scan in order to the effects to take place. The other parameters are correctly set.
        """
        self.__presetting = value

        old_argument_controller = self.argument_controller
        self.argument_controller = ArgumentController('opscan_persistent_data_' + str(self.__presetting))

        for (key, values) in old_argument_controller.argument_controller.items():
            if key not in self.argument_controller.argument_controller:
                self.argument_controller.set(key, values)
            else:
                new_value = self.argument_controller.get(key)
                if new_value != values:
                    self.__last_frame_parameters = None
                    setattr(self, key, new_value)
                    print(f"***OScan***: The new argument controller has a different {key} with value {new_value}.")

        # All the values that are registered in the new pre-setting will be updated
        for (key, _) in self.argument_controller.argument_controller.items():
            self.property_changed_event.fire(key)


    @property
    def imagedisplay(self):
        return self.argument_controller.get('imagedisplay', 0)

    @imagedisplay.setter
    def imagedisplay(self, value):
        self.argument_controller.update(imagedisplay=int(value))

    @property
    def imagedisplay_filter_intensity(self):
        return self.argument_controller.get('imagedisplay_filter_intensity', 0)

    @imagedisplay_filter_intensity.setter
    def imagedisplay_filter_intensity(self, value):
        self.argument_controller.update(imagedisplay_filter_intensity=int(value))

    @property
    def complete_image_offset(self):
        return self.argument_controller.get('complete_image_offset', 0)

    @complete_image_offset.setter
    def complete_image_offset(self, value):
        self.argument_controller.update(complete_image_offset=int(value))
        self.device.set_complete_image_offset(int(value))

    @property
    def image_cumul(self):
        return self.argument_controller.get('image_cumul', 1)

    @image_cumul.setter
    def image_cumul(self, value):
        self.argument_controller.update(image_cumul=int(value))
        self.device.set_image_cumul(int(value))

    @property
    def flyback_us(self):
        return self.argument_controller.get('flyback_us', 0)

    @flyback_us.setter
    def flyback_us(self, value):
        self.argument_controller.update(flyback_us=int(value))
        self._update_frame_parameter()

    @property
    def duty_cycle(self):
        return self.argument_controller.get('duty_cycle', 100)

    @duty_cycle.setter
    def duty_cycle(self, value):
        self.argument_controller.update(duty_cycle=int(value))
        self._update_frame_parameter()

    @property
    def iir_filter(self):
        return self.argument_controller.get('iir_filter', 0)

    @iir_filter.setter
    def iir_filter(self, value):
        self.argument_controller.update(iir_filter=int(value))
        self.device.change_video_parameters(iir_filter=self.argument_controller.get('iir_filter'))

    @property
    def video_delay(self):
        return self.argument_controller.get('video_delay', 0)

    @video_delay.setter
    def video_delay(self, value):
        self.argument_controller.update(video_delay=int(value))
        self.device.change_video_parameters(video_delay=self.video_delay)
        self.property_changed_event.fire("video_phase")
        if self.debug_io:
            self._update_frame_parameter()
        # If timepix3 is present, we should try to set the metadata of this value
        cam = HardwareSource.HardwareSourceManager() \
            .get_hardware_source_for_hardware_source_id("orsay_camera_timepix3")
        if cam is not None:
            cam.camera.camera.set_video_delay(value)

    @property
    def video_phase(self):
        phase = round((self.video_delay / (1.0 / self.lissajous_nx * 1e8)) * 360, 3)
        return phase

    @video_phase.setter
    def video_phase(self, value):
        video_delay = float(value) / 360.0 * (1.0 / self.lissajous_nx * 1e8)
        self.video_delay = video_delay
        self.property_changed_event.fire("video_delay")

    @property
    def pause_sampling(self):
        return self.argument_controller.get('pause_sampling', 0)

    @pause_sampling.setter
    def pause_sampling(self, value):
        self.argument_controller.update(pause_sampling=int(value))
        self.device.change_video_parameters(pause_sampling=self.argument_controller.get('pause_sampling'))

    @property
    def adc_acquisition_mode(self):
        return self.argument_controller.get('adc_acquisition_mode', 5)

    @adc_acquisition_mode.setter
    def adc_acquisition_mode(self, value):
        self.argument_controller.update(adc_acquisition_mode=int(value))
        self.device.change_video_parameters(adc_acquisition_mode=self.argument_controller.get('adc_acquisition_mode'))

    @property
    def rastering_mode(self):
        return self.argument_controller.get('rastering_mode', 0)

    @rastering_mode.setter
    def rastering_mode(self, value):
        self.argument_controller.update(rastering_mode=int(value))
        self._update_frame_parameter()

    @property
    def mini_scan(self):
        return self.argument_controller.get('mini_scan', 2)

    @mini_scan.setter
    def mini_scan(self, value):
        self.argument_controller.update(mini_scan=int(value))
        self._update_frame_parameter()

    @property
    def lissajous_nx(self):
        return self.argument_controller.get('lissajous_nx', 1000)

    @lissajous_nx.setter
    def lissajous_nx(self, value):
        self.argument_controller.update(lissajous_nx=float(value))
        self._update_frame_parameter()

    @property
    def lissajous_ratio(self):
        return self.argument_controller.get('lissajous_ratio', 1.0)

    @lissajous_ratio.setter
    def lissajous_ratio(self, value):
        self.argument_controller.update(lissajous_ratio=float(value))
        self._update_frame_parameter()

    @property
    def lissajous_phase(self):
        return self.argument_controller.get('lissajous_phase', 0)

    @lissajous_phase.setter
    def lissajous_phase(self, value):
        self.argument_controller.update(lissajous_phase=float(value))
        self._update_frame_parameter()

    @property
    def enable_x_delay(self):
        return self.argument_controller.get('enable_x_delay', int(False))

    @enable_x_delay.setter
    def enable_x_delay(self, value: bool):
        self.argument_controller.update(enable_x_delay=int(value))
        self.device.enable_coil_delay(0, int(value))

    @property
    def video_delay_x(self):
        return self.argument_controller.get('video_delay_x', 0)

    @video_delay_x.setter
    def video_delay_x(self, value: int):
        self.argument_controller.update(video_delay_x=int(value))
        self.device.set_coil_delay(0, int(value))

    @property
    def enable_y_delay(self):
        return self.argument_controller.get('enable_y_delay', int(False))

    @enable_y_delay.setter
    def enable_y_delay(self, value: bool):
        self.argument_controller.update(enable_y_delay=int(value))
        self.device.enable_coil_delay(1, int(value))

    @property
    def video_delay_y(self):
        return self.argument_controller.get('video_delay_y', 0)

    @video_delay_y.setter
    def video_delay_y(self, value: int):
        self.argument_controller.update(video_delay_y=int(value))
        self.device.set_coil_delay(1, int(value))

    @property
    def kernel_mode(self):
        return self.argument_controller.get('kernel_mode', 0)

    @kernel_mode.setter
    def kernel_mode(self, value):
        self.argument_controller.update(kernel_mode=int(value))
        self.device.change_video_parameters(adc_acquisition_mode=self.adc_acquisition_mode,
                                            adc_acquisition_mode_name=ADC_READOUT_MODES[self.adc_acquisition_mode],
                                            kernelMode=KERNEL_LIST[self.kernel_mode],
                                            givenPixel=self.given_pixel,
                                            acquisitionCutoff=self.acquisition_cutoff,
                                            acquisitionWindow=self.acquisition_window)
        self._update_frame_parameter()

    @property
    def given_pixel(self):
        return self.argument_controller.get('given_pixel', 0)

    @given_pixel.setter
    def given_pixel(self, value):
        self.argument_controller.update(given_pixel=int(value))
        self.device.change_video_parameters(adc_acquisition_mode=self.adc_acquisition_mode,
                                            adc_acquisition_mode_name=ADC_READOUT_MODES[self.adc_acquisition_mode],
                                            kernelMode=KERNEL_LIST[self.kernel_mode],
                                            givenPixel=self.given_pixel,
                                            acquisitionCutoff=self.acquisition_cutoff,
                                            acquisitionWindow=self.acquisition_window)

    @property
    def acquisition_cutoff(self):
        return self.argument_controller.get('acquisition_cutoff', 100)

    @acquisition_cutoff.setter
    def acquisition_cutoff(self, value):
        self.argument_controller.update(acquisition_cutoff=int(value))
        self.device.change_video_parameters(adc_acquisition_mode=self.adc_acquisition_mode,
                                            adc_acquisition_mode_name=ADC_READOUT_MODES[self.adc_acquisition_mode],
                                            kernelMode=KERNEL_LIST[self.kernel_mode],
                                            givenPixel=self.given_pixel,
                                            acquisitionCutoff=self.acquisition_cutoff,
                                            acquisitionWindow=self.acquisition_window)

    @property
    def acquisition_window(self):
        return self.argument_controller.get('acquisition_window', 0)

    @acquisition_window.setter
    def acquisition_window(self, value):
        self.argument_controller.update(acquisition_window=int(value))
        self.device.change_video_parameters(adc_acquisition_mode=self.adc_acquisition_mode,
                                            adc_acquisition_mode_name=ADC_READOUT_MODES[self.adc_acquisition_mode],
                                            kernelMode=KERNEL_LIST[self.kernel_mode],
                                            givenPixel=self.given_pixel,
                                            acquisitionCutoff=self.acquisition_cutoff,
                                            acquisitionWindow=self.acquisition_window)

    @property
    def magboard_switches(self):
        return self.argument_controller.get('magboard_switches', '010010')

    @magboard_switches.setter
    def magboard_switches(self, value):
        self.argument_controller.update(magboard_switches=str(value))
        self.device.change_magnification_switches(str(value))

    @property
    def offset_adc0(self):
        return self.argument_controller.get('offset_adc0', 0.0)

    @offset_adc0.setter
    def offset_adc0(self, value):
        self.argument_controller.update(offset_adc0=float(value))
        self.device.change_offset_adc('001', float(value), True)

    @property
    def offset_adc1(self):
        return self.argument_controller.get('offset_adc1', 0.0)

    @offset_adc1.setter
    def offset_adc1(self, value):
        self.argument_controller.update(offset_adc1=float(value))
        self.device.change_offset_adc('000', float(value), True)

    @property
    def offset_adc2(self):
        return self.argument_controller.get('offset_adc2', 0.0)

    @offset_adc2.setter
    def offset_adc2(self, value):
        self.argument_controller.update(offset_adc2=float(value))
        self.device.change_offset_adc('011', float(value), False)

    @property
    def offset_adc3(self):
        return self.argument_controller.get('offset_adc3', 0.0)

    @offset_adc3.setter
    def offset_adc3(self, value):
        self.argument_controller.update(offset_adc3=float(value))
        self.device.change_offset_adc('010', float(value), False)

    @property
    def offset_adc4(self):
        return self.argument_controller.get('offset_adc4', 0.0)

    @offset_adc4.setter
    def offset_adc4(self, value):
        self.argument_controller.update(offset_adc4=float(value))
        self.device.change_offset_adc('000', float(value), False)

    @property
    def offset_adc5(self):
        return self.argument_controller.get('offset_adc5', 0.0)

    @offset_adc5.setter
    def offset_adc5(self, value):
        self.argument_controller.update(offset_adc5=float(value))
        self.device.change_offset_adc('001', float(value), False)

    @property
    def multiblock0(self):
        return self.argument_controller.get('multiblock0', 0.0)

    @multiblock0.setter
    def multiblock0(self, value):
        self.argument_controller.update(multiblock0=float(value))
        variables = [self.argument_controller.get('multiblock0'), self.argument_controller.get('multiblock1'),
                     self.argument_controller.get('multiblock2'),
                     self.argument_controller.get('multiblock3')]
        if not any(x is None for x in variables):
            self.device.change_external_calibration(variables)
            self.device.change_external_values(65535)

    @property
    def multiblock1(self):
        return self.argument_controller.get('multiblock1', 0.0)

    @multiblock1.setter
    def multiblock1(self, value):
        self.argument_controller.update(multiblock1=float(value))
        variables = [self.argument_controller.get('multiblock0'), self.argument_controller.get('multiblock1'),
                     self.argument_controller.get('multiblock2'),
                     self.argument_controller.get('multiblock3')]
        if not any(x is None for x in variables):
            self.device.change_external_calibration(variables)
            self.device.change_external_values(65535)

    @property
    def multiblock2(self):
        return self.argument_controller.get('multiblock2', 0.0)

    @multiblock2.setter
    def multiblock2(self, value):
        self.argument_controller.update(multiblock2=float(value))
        variables = [self.argument_controller.get('multiblock0'), self.argument_controller.get('multiblock1'),
                     self.argument_controller.get('multiblock2'),
                     self.argument_controller.get('multiblock3')]
        if not any(x is None for x in variables):
            self.device.change_external_calibration(variables)
            self.device.change_external_values(65535)

    @property
    def multiblock3(self):
        return self.argument_controller.get('multiblock3', 0.0)

    @multiblock3.setter
    def multiblock3(self, value):
        self.argument_controller.update(multiblock3=float(value))
        variables = [self.argument_controller.get('multiblock0'), self.argument_controller.get('multiblock1'),
                     self.argument_controller.get('multiblock2'),
                     self.argument_controller.get('multiblock3')]
        if not any(x is None for x in variables):
            self.device.change_external_calibration(variables)
            self.device.change_external_values(65535)

    @property
    def mag_multiblock0(self):
        return self.argument_controller.get('mag_multiblock0', 1.0)

    @mag_multiblock0.setter
    def mag_multiblock0(self, value):
        self.argument_controller.update(mag_multiblock0=float(value))
        variables = [self.mag_multiblock0, self.mag_multiblock1,
                     self.mag_multiblock2, self.mag_multiblock3,
                     self.mag_multiblock4, self.mag_multiblock5]
        if not any(x is None for x in variables):
            self.device.change_magnification_calibration(variables)

    @property
    def mag_multiblock1(self):
        return self.argument_controller.get('mag_multiblock1', 1.0)

    @mag_multiblock1.setter
    def mag_multiblock1(self, value):
        self.argument_controller.update(mag_multiblock1=float(value))
        variables = [self.mag_multiblock0, self.mag_multiblock1,
                     self.mag_multiblock2, self.mag_multiblock3,
                     self.mag_multiblock4, self.mag_multiblock5]
        if not any(x is None for x in variables):
            self.device.change_magnification_calibration(variables)

    @property
    def mag_multiblock2(self):
        return self.argument_controller.get('mag_multiblock2', 1.0)

    @mag_multiblock2.setter
    def mag_multiblock2(self, value):
        self.argument_controller.update(mag_multiblock2=float(value))
        variables = [self.mag_multiblock0, self.mag_multiblock1,
                     self.mag_multiblock2, self.mag_multiblock3,
                     self.mag_multiblock4, self.mag_multiblock5]
        if not any(x is None for x in variables):
            self.device.change_magnification_calibration(variables)

    @property
    def mag_multiblock3(self):
        return self.argument_controller.get('mag_multiblock3', 1.0)

    @mag_multiblock3.setter
    def mag_multiblock3(self, value):
        self.argument_controller.update(mag_multiblock3=float(value))
        variables = [self.mag_multiblock0, self.mag_multiblock1,
                     self.mag_multiblock2, self.mag_multiblock3,
                     self.mag_multiblock4, self.mag_multiblock5]
        if not any(x is None for x in variables):
            self.device.change_magnification_calibration(variables)

    @property
    def mag_multiblock4(self):
        return self.argument_controller.get('mag_multiblock4', 0.0)

    @mag_multiblock4.setter
    def mag_multiblock4(self, value):
        self.argument_controller.update(mag_multiblock4=float(value))
        variables = [self.mag_multiblock0, self.mag_multiblock1,
                     self.mag_multiblock2, self.mag_multiblock3,
                     self.mag_multiblock4, self.mag_multiblock5]
        if not any(x is None for x in variables):
            self.device.change_magnification_calibration(variables)

    @property
    def mag_multiblock5(self):
        return self.argument_controller.get('mag_multiblock5', 0.0)

    @mag_multiblock5.setter
    def mag_multiblock5(self, value):
        self.argument_controller.update(mag_multiblock5=float(value))
        variables = [self.mag_multiblock0, self.mag_multiblock1,
                     self.mag_multiblock2, self.mag_multiblock3,
                     self.mag_multiblock4, self.mag_multiblock5]
        if not any(x is None for x in variables):
            self.device.change_magnification_calibration(variables)

    @property
    def input1_mux(self):
        return self.argument_controller.get('input1_mux', 0)

    @input1_mux.setter
    def input1_mux(self, value):
        self.argument_controller.update(input1_mux=int(value))
        self.device.set_input_mux(0, int(value))
        self.property_changed_event.fire("input1_mux")

    @property
    def input2_mux(self):
        return self.argument_controller.get('input2_mux', 0)

    @input2_mux.setter
    def input2_mux(self, value):
        self.argument_controller.update(input2_mux=int(value))
        self.device.set_input_mux(1, int(value))
        self.property_changed_event.fire("input2_mux")

    @property
    def input_mux_pwm(self):
        return self.argument_controller.get('input_mux_pwm', 0)

    @input_mux_pwm.setter
    def input_mux_pwm(self, value):
        self.argument_controller.update(input_mux_pwm=int(value))
        self.device.set_input_pwm(int(value))
        self.property_changed_event.fire("input_mux_pwm")

    @property
    def routex_mux(self):
        return self.argument_controller.get('routex_mux', 0)

    @routex_mux.setter
    def routex_mux(self, value):
        self.argument_controller.update(routex_mux=int(value))
        variables = [0, self.routex_mux,
                     self.routex_mux_intensity,
                     self.routex_mux_averages]
        if not any(x is None for x in variables):
            self.device.set_route_mux(*variables)
        self.property_changed_event.fire("routex_mux")

    @property
    def routex_mux_intensity(self):
        return self.argument_controller.get('routex_mux_intensity', 0)

    @routex_mux_intensity.setter
    def routex_mux_intensity(self, value):
        self.argument_controller.update(routex_mux_intensity=int(value))
        variables = [0, self.routex_mux,
                     self.routex_mux_intensity,
                     self.routex_mux_averages]
        if not any(x is None for x in variables):
            self.device.set_route_mux(*variables)
        self.property_changed_event.fire("routex_mux_intensity")

    @property
    def routex_mux_averages(self):
        return self.argument_controller.get('routex_mux_averages', 0)

    @routex_mux_averages.setter
    def routex_mux_averages(self, value):
        self.argument_controller.update(routex_mux_averages=int(value))
        variables = [0, self.routex_mux,
                     self.routex_mux_intensity,
                     self.routex_mux_averages]
        if not any(x is None for x in variables):
            self.device.set_route_mux(*variables)
        self.property_changed_event.fire("routex_mux_averages")

    @property
    def routey_mux(self):
        return self.argument_controller.get('routey_mux', 0)

    @routey_mux.setter
    def routey_mux(self, value):
        self.argument_controller.update(routey_mux=int(value))
        variables = [1, self.routey_mux,
                     self.routey_mux_intensity,
                     self.routey_mux_averages]
        if not any(x is None for x in variables):
            self.device.set_route_mux(*variables)
        self.property_changed_event.fire("routey_mux")

    @property
    def routey_mux_intensity(self):
        return self.argument_controller.get('routey_mux_intensity', 0)

    @routey_mux_intensity.setter
    def routey_mux_intensity(self, value):
        self.argument_controller.update(routey_mux_intensity=int(value))
        variables = [1, self.routey_mux,
                     self.routey_mux_intensity,
                     self.routey_mux_averages]
        if not any(x is None for x in variables):
            self.device.set_route_mux(*variables)
        self.property_changed_event.fire("routey_mux_intensity")

    @property
    def routey_mux_averages(self):
        return self.argument_controller.get('routey_mux_averages', 0)

    @routey_mux_averages.setter
    def routey_mux_averages(self, value):
        self.argument_controller.update(routey_mux_averages=int(value))
        variables = [1, self.routey_mux,
                     self.routey_mux_intensity,
                     self.routey_mux_averages]
        if not any(x is None for x in variables):
            self.device.set_route_mux(*variables)
        self.property_changed_event.fire("routey_mux_averages")

    """
    These are not directly binded, but are convenient for accessing properties
    """
    @property
    def mux_output_type(self):
        return self.argument_controller.get('mux_output_type', [0, 1, 2, 3, 6, 0, 0, 0])

    @mux_output_type.setter
    def mux_output_type(self, value: list):
        self.argument_controller.update(mux_output_type=value)
        for channel in range(5):
            setattr(self, 'mux_output_type'+str(channel+1), value[channel])

    @property
    def mux_output_freq(self):
        return self.argument_controller.get('mux_output_freq', [0] * 8)

    @mux_output_freq.setter
    def mux_output_freq(self, value: list):
        self.argument_controller.update(mux_output_freq=value)
        for channel in range(5):
            setattr(self, 'mux_output_freq'+str(channel+1), value[channel])

    @property
    def mux_output_freq_duty(self):
        return self.argument_controller.get('mux_output_freq_duty', [0] * 8)

    @mux_output_freq_duty.setter
    def mux_output_freq_duty(self, value: list):
        self.argument_controller.update(mux_output_freq_duty=value)
        for channel in range(5):
            setattr(self, 'mux_output_freq_duty'+str(channel+1), value[channel])

    @property
    def mux_output_input(self):
        return self.argument_controller.get('mux_output_input', [0] * 8)

    @mux_output_input.setter
    def mux_output_input(self, value: list):
        self.argument_controller.update(mux_output_input=value)
        for channel in range(5):
            setattr(self, 'mux_output_input'+str(channel+1), value[channel])

    @property
    def mux_output_input_div(self):
        return self.argument_controller.get('mux_output_input_div', [0] * 8)

    @mux_output_input_div.setter
    def mux_output_input_div(self, value: list):
        self.argument_controller.update(mux_output_input_div=value)
        for channel in range(5):
            setattr(self, 'mux_output_input_div'+str(channel+1), value[channel])

    @property
    def mux_output_delay(self):
        return self.argument_controller.get('mux_output_delay', [0] * 8)

    @mux_output_delay.setter
    def mux_output_delay(self, value: list):
        self.argument_controller.update(mux_output_delay=value)
        for channel in range(5):
            setattr(self, 'mux_output_delay'+str(channel+1), value[channel])

    @property
    def mux_output_pol(self):
        return self.argument_controller.get('mux_output_pol', [0] * 8)

    @mux_output_pol.setter
    def mux_output_pol(self, value: list):
        self.argument_controller.update(mux_output_pol=value)
        for channel in range(5):
            setattr(self, 'mux_output_pol'+str(channel+1), value[channel])

    """
    Creating the functions for the output multiplexer. I have used the property function for efficiency
    """
    def _get_variables(self, channel: int):
        """
        This is used internally for convenience. The function set_output_mux updates all arguments at once
        """
        variables = [channel, self.argument_controller.get('mux_output_type', [0] * 8)[channel],
                     self.argument_controller.get('mux_output_freq', [0] * 8)[channel],
                     self.argument_controller.get('mux_output_freq_duty', [0] * 8)[channel],
                     self.argument_controller.get('mux_output_input', [0] * 8)[channel],
                     self.argument_controller.get('mux_output_input_div', [0] * 8)[channel],
                     self.argument_controller.get('mux_output_delay', [0] * 8)[channel],
                     self.argument_controller.get('mux_output_pol', [0] * 8)[channel]]
        return variables

    def _create_get_func(channel: int, identifier: str):
        def wrapper(self):
            return self.argument_controller.get(identifier, [0] * 8)[channel]
        return wrapper

    def _create_set_func(channel: int, identifier: str):
        def wrapper(self, value):
            temp_list = self.argument_controller.get(identifier, [0] * 8)
            if "freq" in identifier: #The two frequency terms can be float.
                temp_list[channel] = float(value)
            else:
                temp_list[channel] = int(value)
            variables = self._get_variables(channel)
            self.device.set_output_mux(*variables)
            self.argument_controller.set(identifier, temp_list)
            self.property_changed_event.fire(identifier+str(channel))
        return wrapper

    mux_output_type1 = property(_create_get_func(0, 'mux_output_type'), _create_set_func(0, 'mux_output_type'))
    mux_output_freq1 = property(_create_get_func(0, 'mux_output_freq'), _create_set_func(0, 'mux_output_freq'))
    mux_output_freq_duty1 = property(_create_get_func(0, 'mux_output_freq_duty'),
                                     _create_set_func(0, 'mux_output_freq_duty'))
    mux_output_input1 = property(_create_get_func(0, 'mux_output_input'), _create_set_func(0, 'mux_output_input'))
    mux_output_input_div1 = property(_create_get_func(0, 'mux_output_input_div'), _create_set_func(0, 'mux_output_input_div'))
    mux_output_delay1 = property(_create_get_func(0, 'mux_output_delay'), _create_set_func(0, 'mux_output_delay'))
    mux_output_pol1 = property(_create_get_func(0, 'mux_output_pol'), _create_set_func(0, 'mux_output_pol'))

    mux_output_type2 = property(_create_get_func(1, 'mux_output_type'), _create_set_func(1, 'mux_output_type'))
    mux_output_freq2 = property(_create_get_func(1, 'mux_output_freq'), _create_set_func(1, 'mux_output_freq'))
    mux_output_freq_duty2 = property(_create_get_func(1, 'mux_output_freq_duty'),
                                     _create_set_func(1, 'mux_output_freq_duty'))
    mux_output_input2 = property(_create_get_func(1, 'mux_output_input'), _create_set_func(1, 'mux_output_input'))
    mux_output_input_div2 = property(_create_get_func(1, 'mux_output_input_div'), _create_set_func(1, 'mux_output_input_div'))
    mux_output_delay2 = property(_create_get_func(1, 'mux_output_delay'), _create_set_func(1, 'mux_output_delay'))
    mux_output_pol2 = property(_create_get_func(1, 'mux_output_pol'), _create_set_func(1, 'mux_output_pol'))

    mux_output_type3 = property(_create_get_func(2, 'mux_output_type'), _create_set_func(2, 'mux_output_type'))
    mux_output_freq3 = property(_create_get_func(2, 'mux_output_freq'), _create_set_func(2, 'mux_output_freq'))
    mux_output_freq_duty3 = property(_create_get_func(2, 'mux_output_freq_duty'),
                                     _create_set_func(2, 'mux_output_freq_duty'))
    mux_output_input3 = property(_create_get_func(2, 'mux_output_input'), _create_set_func(2, 'mux_output_input'))
    mux_output_input_div3 = property(_create_get_func(2, 'mux_output_input_div'), _create_set_func(2, 'mux_output_input_div'))
    mux_output_delay3 = property(_create_get_func(2, 'mux_output_delay'), _create_set_func(2, 'mux_output_delay'))
    mux_output_pol3 = property(_create_get_func(2, 'mux_output_pol'), _create_set_func(2, 'mux_output_pol'))

    mux_output_type4 = property(_create_get_func(3, 'mux_output_type'), _create_set_func(3, 'mux_output_type'))
    mux_output_freq4 = property(_create_get_func(3, 'mux_output_freq'), _create_set_func(3, 'mux_output_freq'))
    mux_output_freq_duty4 = property(_create_get_func(3, 'mux_output_freq_duty'),
                                     _create_set_func(3, 'mux_output_freq_duty'))
    mux_output_input4 = property(_create_get_func(3, 'mux_output_input'), _create_set_func(3, 'mux_output_input'))
    mux_output_input_div4 = property(_create_get_func(3, 'mux_output_input_div'), _create_set_func(3, 'mux_output_input_div'))
    mux_output_delay4 = property(_create_get_func(3, 'mux_output_delay'), _create_set_func(3, 'mux_output_delay'))
    mux_output_pol4 = property(_create_get_func(3, 'mux_output_pol'), _create_set_func(3, 'mux_output_pol'))

    mux_output_type5 = property(_create_get_func(4, 'mux_output_type'), _create_set_func(4, 'mux_output_type'))
    mux_output_freq5 = property(_create_get_func(4, 'mux_output_freq'), _create_set_func(4, 'mux_output_freq'))
    mux_output_freq_duty5 = property(_create_get_func(4, 'mux_output_freq_duty'),
                                     _create_set_func(4, 'mux_output_freq_duty'))
    mux_output_input5 = property(_create_get_func(4, 'mux_output_input'), _create_set_func(4, 'mux_output_input'))
    mux_output_input_div5 = property(_create_get_func(4, 'mux_output_input_div'), _create_set_func(4, 'mux_output_input_div'))
    mux_output_delay5 = property(_create_get_func(4, 'mux_output_delay'), _create_set_func(4, 'mux_output_delay'))
    mux_output_pol5 = property(_create_get_func(4, 'mux_output_pol'), _create_set_func(4, 'mux_output_pol'))


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


class Device(scan_base.ScanDevice):
    def __init__(self, instrument):
        self.scan_device_id = "open_scan_device"
        self.scan_device_is_secondary = True
        self.scan_device_name = _("OpScan")
        self.__channels = self.__get_channels()
        self.__frame = None
        self.__frame_number = 0
        self.__instrument = instrument
        self.__probe_position = Geometry.FloatPoint(0.5, 0.5)
        self.__is_scanning = False
        self.on_device_state_changed = None
        self.flyback_pixels = 2
        self.__buffer = list()
        self.__sequence_buffer_size = 0
        self.__view_buffer_size = 20
        self.bottom_blanker = 0
        self.scan_engine = ScanEngine()

        self.has_data_event = threading.Event()

    def close(self):
        pass

    def stop(self) -> None:
        """Stop acquiring."""
        if self.__is_scanning:
            self.scan_engine.set_probe_position(self.__probe_position.x, self.__probe_position.y)
            self.__is_scanning = False

    def set_idle_position_by_percentage(self, x: float, y: float) -> None:
        """Set the idle position as a percentage of the last used frame parameters."""
        pass

    def cancel(self) -> None:
        """Cancel acquisition (immediate)."""
        if self.__is_scanning:
            self.scan_engine.set_probe_position(self.__probe_position.x, self.__probe_position.y)
            self.__is_scanning = False

    def __get_channels(self) -> typing.List[Channel]:
        channels = [Channel(0, "ListScan", False), Channel(1, "BF", False), Channel(2, "ADF", True)]
        return channels

    # def __get_initial_profiles(self) -> typing.List[scan_base.ScanFrameParameters]:
    #     profiles = list()
    #     profiles.append(scan_base.ScanFrameParameters(
    #         {"size": (128, 128), "pixel_time_us": 0.5, "fov_nm": 4000., "rotation_rad": 0.393}))
    #     profiles.append(scan_base.ScanFrameParameters({"size": (128, 128), "pixel_time_us": 1, "fov_nm": 100.}))
    #     profiles.append(scan_base.ScanFrameParameters({"size": (512, 512), "pixel_time_us": 1, "fov_nm": 100.}))
    #     return profiles

    def get_channel_name(self, channel_index: int) -> str:
        return self.__channels[channel_index].name

    def set_frame_parameters(self, frame_parameters: scan_base.ScanFrameParameters) -> None:
        """Called just before and during acquisition.
        Device should use these parameters for new acquisition; and update to these parameters during acquisition.
        """
        self.scan_engine.set_frame_parameters(frame_parameters)
        self.__frame_parameters = copy.deepcopy(frame_parameters)

    def save_frame_parameters(self) -> None:
        """Called when shutting down. Save frame parameters to persistent storage."""
        pass

    def get_current_image_size(self, frame_parameters: scan_base.ScanFrameParameters) -> tuple:
        value = frame_parameters.get_parameter("subscan_pixel_size", None)
        if value == None:
            value = frame_parameters.as_dict()["pixel_size"]
        return value

    def start_frame(self, is_continuous: bool) -> int:
        """Start acquiring. Return the frame number."""
        if not self.__is_scanning:
            self.__buffer = list()
            self.__start_next_frame()
            self.__is_scanning = True
        return self.__frame_number

    def __start_next_frame(self):
        frame_parameters = copy.deepcopy(self.__frame_parameters)
        self.__scan_context = stem_controller.ScanContext()
        channels = [copy.deepcopy(channel) for channel in self.__channels if channel.enabled]  # channel enabled is here
        size = Geometry.IntSize.make(
            frame_parameters.subscan_pixel_size if frame_parameters.subscan_pixel_size else frame_parameters.size)
        for channel in channels:
            channel.data = numpy.zeros(tuple(size), numpy.float32)
        is_synchronized_scan = frame_parameters.get_parameter("external_clock_mode", 0) != 0
        self.__frame_number = self.scan_engine.device.get_frame_counter()
        self.__start_frame = self.__frame_number
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

        if self.__frame is None:
            self.__start_next_frame()

        current_frame = self.__frame  # this is from Frame Class defined above
        assert current_frame is not None
        # frame_number = current_frame.frame_number
        self.__frame_number = self.scan_engine.device.get_frame_counter()
        is_synchronized_scan = current_frame.frame_parameters.get_parameter("external_clock_mode", 0) != 0
        if is_synchronized_scan:
            time.sleep(TIMEOUT_IS_SYNC)
            current_frame.complete = self.scan_engine.device.get_dma_status_idle()[0] == 2
            if current_frame.complete:
                sub_area = ((0, 0), self.get_current_image_size(current_frame.frame_parameters))
            else:
                (x, y) = self.get_current_image_size(current_frame.frame_parameters)
                lines = self.scan_engine.device.get_pixel_counter() // y
                sub_area = ((0, 0), (lines, y))
        else:
            current_frame.complete = self.__frame_number != self.__start_frame
            sub_area = ((0, 0), self.get_current_image_size(current_frame.frame_parameters))

        if DEBUG:
            print(
                f"{current_frame.complete} and {sub_area} and {self.scan_engine.device.get_pixel_counter()} and {self.__frame_number} and {self.__start_frame} "
                f"and {self.scan_engine.device.get_dma_status_idle()} and {self.scan_engine.device.get_bd_status_cmplt()} "
                f"and {self.get_current_image_size(current_frame.frame_parameters)}")

        data_elements = list()

        for channel in current_frame.channels:
            data_element = dict()
            data_array = self.scan_engine.receive_total_frame(channel.channel_id)
            data_element["data"] = data_array
            properties = current_frame.frame_parameters.as_dict()
            properties["center_x_nm"] = current_frame.frame_parameters.center_nm[1]
            if is_synchronized_scan:
                properties["decode_list"] = self.scan_engine.get_mask_array().tolist()
            properties["center_y_nm"] = current_frame.frame_parameters.center_nm[0]
            properties["rotation_deg"] = math.degrees(current_frame.frame_parameters.rotation_rad)
            properties["channel_id"] = channel.channel_id

            #Properties from scan_engine that must be updated in a frame_base
            self.scan_engine.update_metadata_to_dict(properties)

            data_element["properties"] = properties
            if data_array is not None:
                data_elements.append(data_element)

        if current_frame.complete:
            if len(self.__buffer) > 0 and len(self.__buffer[-1]) != len(data_elements):
                self.__buffer = list()
            self.__buffer.append(data_elements)
            while len(self.__buffer) > self.__sequence_buffer_size + self.__view_buffer_size:
                del self.__buffer[self.__sequence_buffer_size]
            self.__frame = None

        # return data_elements, complete, bad_frame, sub_area, frame_number, pixels_to_skip
        return data_elements, current_frame.complete, False, sub_area, self.__frame_number, 0

    # This one is called in scan_base
    def prepare_synchronized_scan(self, scan_frame_parameters: scan_base.ScanFrameParameters, *, camera_exposure_ms,
                                  **kwargs) -> None:
        # scan_frame_parameters.set_parameter("pixel_time_us", int(1000 * camera_exposure_ms))
        scan_frame_parameters.set_parameter("external_clock_mode", 1)

    def set_sequence_buffer_size(self, buffer_size: int) -> None:
        self.__sequence_buffer_size = buffer_size
        self.__buffer = list()

    def get_sequence_buffer_count(self) -> int:
        return len(self.__buffer)

    def pop_sequence_buffer_data(self) -> typing.List[typing.Dict[str, typing.Any]]:
        self.__sequence_buffer_size -= 1
        return self.__buffer.pop(0)

    def get_buffer_data(self, start: int, count: int) -> typing.List[typing.List[typing.Dict[str, typing.Any]]]:
        # print(f"get {start=} {count=} {len(self.__buffer)=}")
        # time.sleep(0.1)
        if start < 0:
            return self.__buffer[start: start + count if count < -start else None]
        else:
            return self.__buffer[start: start + count]

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

    @property
    def pixel_time(self):
        return self.__pixeltime

    @pixel_time.setter
    def pixel_time(self, value):
        self.__pixeltime = value / 1e6

    @property
    def scan_rotation(self):
        frame_parameters = copy.deepcopy(self.__frame_parameters)
        return frame_parameters.rotation_rad * 180.0 / numpy.pi

    @property
    def Image_area(self):
        return self.__scan_area

    @Image_area.setter
    def Image_area(self, value):
        self.__scan_area = value

    @property
    def probe_pos(self):
        return self.__probe_position

    @probe_pos.setter
    def probe_pos(self, value):
        self.__probe_position = value
        self.scan_engine.set_probe_position(value.x, value.y)

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

class ScanSettings(scan_base.ScanSettings):

    def __init__(self, scan_modes, frame_parameters_factory, current_settings_index=0, record_settings_index=0,
                 open_configuration_dialog_fn=None) -> None:
        super(ScanSettings, self).__init__(scan_modes, frame_parameters_factory, current_settings_index,
                                           record_settings_index, open_configuration_dialog_fn)

    def open_configuration_interface(self, api_broker: typing.Any) -> None:
        if callable(self.__open_configuration_dialog_fn):
            self.__open_configuration_dialog_fn(api_broker)


class ScanModule(scan_base.ScanModule):
    def __init__(self, instrument) -> None:
        self.stem_controller_id = instrument.instrument_id
        self.device = Device(instrument)
        setattr(self.device, "priority", 20)
        scan_modes = generate_initial_scan_profiles()
        self.settings = ScanSettings(scan_modes, lambda d: scan_base.ScanFrameParameters(d), 0, 2,
                                     open_configuration_dialog_fn=show_configuration_dialog)


def show_configuration_dialog(api_broker) -> None:
    """Open settings dialog, if any."""
    api = api_broker.get_api(version="1", ui_version="1")
    document_controller = api.application.document_controllers[0]._document_controller
    myConfig = ConfigDialog(document_controller)


def run(instrument) -> None:
    Registry.register_component(ScanModule(instrument), {"scan_module"})


def generate_initial_scan_profiles():
    return (scan_base.ScanSettingsMode(_("Fast"), "fast",
                                       scan_base.ScanFrameParameters(pixel_size=(128, 128), pixel_time_us=1,
                                                                 fov_nm=1000)),
        scan_base.ScanSettingsMode(_("Slow"), "slow",
                                   scan_base.ScanFrameParameters(pixel_size=(512, 512), pixel_time_us=1,
                                                                 fov_nm=1000)),
        scan_base.ScanSettingsMode(_("Record"), "record",
                                   scan_base.ScanFrameParameters(pixel_size=(1024, 1024), pixel_time_us=1,
                                                                 fov_nm=1000))
            )


def stop() -> None:
    Registry.unregister_component(Registry.get_component("scan_module"), {"scan_module"})
