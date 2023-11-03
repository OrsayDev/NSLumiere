# standard libraries
import copy, math, gettext, numpy, typing, time, socket, threading, logging, sys
sys.path.append('/home/yvesauad/Documents/OpenScanPY')

# local libraries
from nion.utils import Registry
from nion.utils import Geometry
from nion.instrumentation import scan_base
from nion.instrumentation import stem_controller

from nionswift_plugin.IVG.scan.OScanCesysDialog import ConfigDialog

from nionswift_plugin.IVG import ivg_inst
import FPGAConfig

_ = gettext.gettext

BUFFER_SIZE = 2048
PACKET_LEN = 1024
CLOCK_FREQUENCY = 100
MAX_BUFFER_DESCRIPTORS = 1
CLOCK_TICK = 1 / CLOCK_FREQUENCY * 1e3  # in nanoseconds

class ScanEngine:
    def __init__(self):

        try:
            io_system = FPGAConfig.CesysDevice("/home/yvesauad/Documents/NSLumiere/nionswift_plugin/IVG/scan/".encode(),
                                               "/home/yvesauad/FPGA_Projects/efm03_reference_design-v1.2.1/EFM03_exdes/EFM03_exdes.runs/impl_1/efm03_wrapper.bin")
            self.device = FPGAConfig.ScanDevice(io_system, 128, 128, 100, 0)
        except UnboundLocalError:
            io_system = FPGAConfig.DebugClass()
            self.device = FPGAConfig.ScanDevice(io_system, 128, 128, 100, 0)
            logging.warning(f'Could not found CESYS system connected. Entering in debug mode.')
        except:
            io_system = FPGAConfig.DebugClass()
            self.device = FPGAConfig.ScanDevice(io_system, 128, 128, 100, 0)
            logging.warning(f'Could not found the library libudk3-1.5.1.so. You should probably use '
                            f'export LD_LIBRARY_PATH=/home/yvesauad/Documents/NSLumiere/nionswift_plugin/IVG/scan')


        self.__x = None
        self.__y = None
        self.__pixel_ratio = None

        #Settings
        self.__imagedisplay = None
        self.__dsp_filter = None
        self.__external_trigger = None
        self.__flyback_us = None
        self.__rastering_mode = None
        self.__magboard_switches = None
        self.__offset_adc = None
        self.__lissajous_phase = None
        self.__lissajous_nx = None
        self.__lissajous_ny = None
        self.__kernel_mode = None
        self.__acquisition_cutoff = None
        self.__acquisition_window = None

        self.__imagedisplay = 0
        self.__adc_mode = 0
        self.dsp_filter = 0
        self.external_trigger = 0
        self.flyback_us = 0
        self.rastering_mode = 0
        self.magboard_switches = '100100'
        self.offset_adc = 13000
        self.lissajous_nx = 1
        self.lissajous_ny = 1
        self.lissajous_phase = 0
        self.kernel_mode = 0
        self.acquisition_cutoff = 500
        self.acquisition_window = 2

    def receive_total_frame(self, channel: int):
        image = self.device.get_image(channel, imageType = self.imagedisplay)
        return image

    def set_frame_parameters(self, x, y, pixel_us, fov_nm):
        self.device.change_scan_parameters(x, y, pixel_us, self.__flyback_us, fov_nm, self.rastering_mode,
                                           lissajous_nx = self.lissajous_nx,
                                           lissajous_ny=self.lissajous_ny,
                                           lissajous_phase = self.lissajous_phase,
                                           kernelMode = self.kernel_mode,
                                           acquisitionCutoff=self.acquisition_cutoff,
                                           acquisitionWindow=self.acquisition_window
                                           )

    def set_probe_position(self, x, y):
        self.device.set_probe_position(x, y)

    @property
    def imagedisplay(self):
        return self.__imagedisplay

    @imagedisplay.setter
    def imagedisplay(self, value):
        if self.__imagedisplay != value:
            self.__imagedisplay = int(value)
    @property
    def flyback_us(self):
        return self.__flyback_us

    @flyback_us.setter
    def flyback_us(self, value):
        if self.__flyback_us != value:
            self.__flyback_us = int(value)

    @property
    def external_trigger(self):
        return self.__external_trigger

    @external_trigger.setter
    def external_trigger(self, value):
        if self.__external_trigger != value:
            self.__external_trigger = value

    @property
    def adc_acquisition_mode(self):
        return self.__adc_mode

    @adc_acquisition_mode.setter
    def adc_acquisition_mode(self, value):
        if self.__adc_mode != value:
            self.__adc_mode = value
            self.device.change_video_acquisition_mode(value)

    @property
    def dsp_filter(self):
        return self.__dsp_filter

    @dsp_filter.setter
    def dsp_filter(self, value):
        if self.__dsp_filter != value:
            self.__dsp_filter = value
            self.device.change_video_parameters(value)

    @property
    def rastering_mode(self):
        return self.__rastering_mode

    @rastering_mode.setter
    def rastering_mode(self, value):
        if self.__rastering_mode != value:
            self.__rastering_mode = value

    @property
    def lissajous_nx(self):
        return self.__lissajous_nx

    @lissajous_nx.setter
    def lissajous_nx(self, value):
        if self.__lissajous_nx != value:
            self.__lissajous_nx = value

    @property
    def lissajous_ny(self):
        return self.__lissajous_ny

    @lissajous_ny.setter
    def lissajous_ny(self, value):
        if self.__lissajous_ny != value:
            self.__lissajous_ny = value

    @property
    def lissajous_phase(self):
        return self.__lissajous_phase

    @lissajous_phase.setter
    def lissajous_phase(self, value):
        if self.__lissajous_phase != value:
            self.__lissajous_phase = value

    @property
    def kernel_mode(self):
        return self.__kernel_mode

    @kernel_mode.setter
    def kernel_mode(self, value):
        if self.__kernel_mode != value:
            self.__kernel_mode = value
            self.device.change_adc_kernel(kernelMode=self.__kernel_mode,
                                          acquisitionCutoff=self.__acquisition_cutoff,
                                          acquisitionWindow=self.__acquisition_window)

    @property
    def acquisition_cutoff(self):
        return self.__acquisition_cutoff

    @acquisition_cutoff.setter
    def acquisition_cutoff(self, value):
        if self.__acquisition_cutoff != value:
            self.__acquisition_cutoff = value
            self.device.change_adc_kernel(kernelMode = self.__kernel_mode,
                                          acquisitionCutoff = self.__acquisition_cutoff,
                                          acquisitionWindow = self.__acquisition_window)

    @property
    def acquisition_window(self):
        return self.__acquisition_window

    @acquisition_window.setter
    def acquisition_window(self, value):
        if self.__acquisition_window != value:
            self.__acquisition_window = value
            self.device.change_adc_kernel(kernelMode=self.__kernel_mode,
                                          acquisitionCutoff=self.__acquisition_cutoff,
                                          acquisitionWindow=self.__acquisition_window)

    @property
    def magboard_switches(self):
        return self.__magboard_switches

    @magboard_switches.setter
    def magboard_switches(self, value):
        if self.__magboard_switches != value:
            self.__magboard_switches = value
            self.device.change_magnification_switches(self.__magboard_switches)

    @property
    def offset_adc(self):
        return self.__offset_adc

    @offset_adc.setter
    def offset_adc(self, value):
        if self.__offset_adc != value:
            self.__offset_adc = int(value)
            self.device.change_offset_adc(self.__offset_adc)

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


class Device:

    def __init__(self, instrument: ivg_inst.ivgInstrument):
        self.scan_device_id = "open_scan_device"
        self.scan_device_name = _("OpScan")
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
        self.flyback_pixels = 0
        self.__buffer = list()
        self.bottom_blanker = 0
        self.scan_engine = ScanEngine()

        self.has_data_event = threading.Event()

    def close(self):
        pass

    def stop(self) -> None:
        """Stop acquiring."""
        if self.__is_scanning:
            self.__is_scanning = False

    def set_idle_position_by_percentage(self, x: float, y: float) -> None:
        """Set the idle position as a percentage of the last used frame parameters."""
        pass

    def cancel(self) -> None:
        """Cancel acquisition (immediate)."""
        if self.__is_scanning:
            self.__is_scanning = False

    def __get_channels(self) -> typing.List[Channel]:
        channels = [Channel(0, "ADF", True), Channel(1, "BF", False), Channel(2, "Hyperspectral", False), Channel(3, "Sync", False)]
        return channels

    def __get_initial_profiles(self) -> typing.List[scan_base.ScanFrameParameters]:
        profiles = list()
        profiles.append(scan_base.ScanFrameParameters(
            {"size": (128, 128), "pixel_time_us": 0.5, "fov_nm": 4000., "rotation_rad": 0.393}))
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
        (x, y) = frame_parameters.as_dict()['pixel_size']
        pixel_time = frame_parameters.as_dict()['pixel_time_us']
        fov_nm = frame_parameters.as_dict()['fov_nm']
        self.scan_engine.set_frame_parameters(x, y, pixel_time, fov_nm)

    def save_frame_parameters(self) -> None:
        """Called when shutting down. Save frame parameters to persistent storage."""
        pass

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
        self.__frame_number += 1  # This is updated in the self.__frame_number
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

        # gotit = self.has_data_event.wait(1.0)

        if self.__frame is None:
            self.__start_next_frame()

        current_frame = self.__frame  # this is from Frame Class defined above
        assert current_frame is not None
        data_elements = list()

        for channel in current_frame.channels:
            data_element = dict()
            data_array = self.scan_engine.receive_total_frame(channel.channel_id)
            data_element["data"] = data_array
            properties = current_frame.frame_parameters.as_dict()
            properties["center_x_nm"] = current_frame.frame_parameters.center_nm[1]
            properties["center_y_nm"] = current_frame.frame_parameters.center_nm[0]
            properties["rotation_deg"] = math.degrees(current_frame.frame_parameters.rotation_rad)
            properties["channel_id"] = channel.channel_id
            data_element["properties"] = properties
            if data_array is not None:
                data_elements.append(data_element)

        # self.has_data_event.clear()

        current_frame.complete = True
        if current_frame.complete:
            self.__frame = None

        # return data_elements, complete, bad_frame, sub_area, frame_number, pixels_to_skip
        return data_elements, current_frame.complete, False, ((0, 0), data_array.shape), None, 0

    # This one is called in scan_base
    def prepare_synchronized_scan(self, scan_frame_parameters: scan_base.ScanFrameParameters, *, camera_exposure_ms,
                                  **kwargs) -> None:
        pass

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
        return self.__rotation

    @scan_rotation.setter
    def scan_rotation(self, value):
        self.__rotation = value * 180 / numpy.pi

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
        # This is a very strange behavior of geometry class. Very misleading. Value is a tuple like
        # (x=0.1, y=0.3) but value[0] gives y value while value[1] gives x value. You can check here
        # print(f'value is {value} and first is {value[0]}. Second is {value[1]}')
        # If you using this func, please call it with (y, x)
        #px, py = round(self.__probe_position[1] * self.__scan_area[1]), round(
        #    self.__probe_position[0] * self.__scan_area[0])
        #self.__probe_position_pixels = [px, py]

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

    def show_configuration_dialog(self, api_broker) -> None:
        """Open settings dialog, if any."""
        #api = api_broker.get_api(version="1", ui_version="1")
        #document_controller = api.application.document_controllers[0]._document_controller
        #myConfig = ConfigDialog(document_controller)


# def run(instrument: ivg_inst.ivgInstrument):
#    scan_device = Device(instrument)
#    component_types = {"scan_device"}  # the set of component types that this component represents
#    Registry.register_component(scan_device, component_types)

class ScanSettings(scan_base.ScanSettings):

    def __init__(self, scan_modes, frame_parameters_factory, current_settings_index = 0, record_settings_index = 0, open_configuration_dialog_fn = None) -> None:
        super(ScanSettings, self).__init__(scan_modes, frame_parameters_factory, current_settings_index, record_settings_index, open_configuration_dialog_fn)

    def open_configuration_interface(self, api_broker: typing.Any) -> None:
        if callable(self.__open_configuration_dialog_fn):
            self.__open_configuration_dialog_fn(api_broker)


class ScanModule(scan_base.ScanModule):
    def __init__(self, instrument: ivg_inst.ivgInstrument) -> None:
        self.stem_controller_id = instrument.instrument_id
        self.device = Device(instrument)
        setattr(self.device, "priority", 20)
        scan_modes = (
            scan_base.ScanSettingsMode(_("Fast"), "fast",
                                       scan_base.ScanFrameParameters(pixel_size=(128, 128), pixel_time_us=1,
                                                                     fov_nm=4000)),
            scan_base.ScanSettingsMode(_("Slow"), "slow",
                                       scan_base.ScanFrameParameters(pixel_size=(512, 512), pixel_time_us=1,
                                                                     fov_nm=4000)),
            scan_base.ScanSettingsMode(_("Record"), "record",
                                       scan_base.ScanFrameParameters(pixel_size=(1024, 1024), pixel_time_us=1,
                                                                     fov_nm=4000))
        )
        self.settings = ScanSettings(scan_modes, lambda d: scan_base.ScanFrameParameters(d), 0, 2,
                                     open_configuration_dialog_fn=show_configuration_dialog)



def show_configuration_dialog(api_broker) -> None:
    """Open settings dialog, if any."""
    api = api_broker.get_api(version="1", ui_version="1")
    document_controller = api.application.document_controllers[0]._document_controller
    myConfig = ConfigDialog(document_controller)


def run(instrument: ivg_inst.ivgInstrument) -> None:
    Registry.register_component(ScanModule(instrument), {"scan_module"})


def stop() -> None:
    Registry.unregister_component(Registry.get_component("scan_module"), {"scan_module"})
