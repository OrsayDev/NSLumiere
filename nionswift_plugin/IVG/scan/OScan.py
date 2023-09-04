# standard libraries
import copy
import math
import gettext
import numpy
import threading
import typing
import time
import socket, threading
import logging

# local libraries
from nion.utils import Registry
from nion.utils import Geometry
from nion.instrumentation import scan_base
from nion.instrumentation import stem_controller

from nionswift_plugin.IVG.scan.OScanDialog import ConfigDialog

from nionswift_plugin.IVG import ivg_inst

_ = gettext.gettext

BUFFER_SIZE = 2048
PACKET_LEN = 1024
CLOCK_FREQUENCY = 100
MAX_BUFFER_DESCRIPTORS = 1
CLOCK_TICK = 1 / CLOCK_FREQUENCY * 1e3  # in nanoseconds


def message_creator(write, initial_address, final_address, value):
    str_write = str(1) if write else str(0)
    str_initial_address = str.zfill(str(initial_address), 8)
    str_final_address = str.zfill(str(final_address), 8)
    str_value = str.zfill(str(value), 10)
    return ('>' + str_write + ',' + str_initial_address + ',' + str_final_address + ',' + str_value).encode()


class ScanListEngine():
    def __init__(self, sock, number_of_points: int):
        self.sock = sock
        self.__number_of_points = number_of_points
        self.__eof = number_of_points * 8

    def set_memory_block_values(self, initial_address: int, final_address: int, value: int) -> None:
        message = message_creator(1, initial_address, final_address, value)
        send = self.sock.send(message)
        assert (send == 31)

    def get_memory_block_value_sampling(self, address: int):
        message = message_creator(0, address, address+4, 0)
        send = self.sock.send(message)
        response = self.sock.recv(512)
        response_decoded = int.from_bytes(response, "little")
        assert (send == 31)
        return response_decoded

    #def erase_values(self):
    #    self.set_memory_block_values(0, 0x100000, 0)

    #def _create_pixel_value(self, index: int, x: int, y: int, pause_time: int, pixel_time: int, eof: bool,
    #                         pulsein: bool, pulseout: bool):
    #    value = (x & 65535) | (y & 65535) << 16 | (pause_time & 65535) << 32 | (pixel_time & 8191) << 48 | (
    #                eof & 1) << 61 | (pulsein & 1) << 62 | (pulseout & 1) << 63
    #    value1 = (x & 65535) | (y & 65535) << 16
    #    value2 = (pause_time & 65535) | (pixel_time & 8191) << 16 | (
    #                eof & 1) << 29 | (pulsein & 1) << 30 | (pulseout & 1) << 31
    #    self.set_memory_block_values(index, index+4, value1)
    #    self.set_memory_block_values(index+4, index + 8, value2)
    #    #self.set_memory_block_values(index, index + 8, value)

    """
    def set_eof_value(self, address: int):
        #print(self.__eof)
        #if self.__eof:
        current_value = self.get_memory_block_value_sampling(address * 8 - 4)
        #pause_time = current_value & 65535
        #pixel_time = (current_value >> 16 ) & 8191
        #eof = (current_value >> 29) & 1
        #pulsein = (current_value >> 30) & 1
        #pulseout = (current_value >> 31) & 1
        new_value = current_value | 0x20000000
        #print(pause_time, pixel_time, eof, pulsein, pulseout)
        self.set_memory_block_values(address * 8 - 4, address * 8, new_value)
    """

    def create_square_list(self, xmax: int, ymax: int, pixel_time: int, hyperspec: bool, rastering_mode: int):
        value1 = ((xmax & 4095) | (ymax & 4095) << 12)
        value2 = pixel_time << 1 | hyperspec & 1
        mode = 0
        if rastering_mode == 1:
            mode = mode | 1
        if rastering_mode == 2:
            mode = mode | 2
        self.set_memory_block_values(value1, value2, mode)

class ScanEngine:
    def __init__(self):
        #self.__lock = threading.Lock()

        self.scan_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.scan_sock.connect(("192.168.1.10", 98))
        self.scan_sock.send(b'scan_device')
        time.sleep(0.01)

        self.video_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.video_sock.connect(("192.168.1.10", 98))
        self.video_sock.send(b'video_device')
        time.sleep(0.01)

        self.spim_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.spim_sock.connect(("192.168.1.10", 98))
        self.spim_sock.send(b'dma_spim')
        time.sleep(0.01)

        self.mem_list = list()

        mem_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        mem_sock.connect(("192.168.1.10", 98))
        mem_sock.send(b'memory_block_00')
        mem_sock.settimeout(0.5)
        self.mem_list.append(mem_sock)
        time.sleep(0.01)

        mem_sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        mem_sock2.connect(("192.168.1.10", 98))
        mem_sock2.send(b'memory_block_01')
        mem_sock2.settimeout(0.5)
        self.mem_list.append(mem_sock2)
        time.sleep(0.01)

        mem_sock3 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        mem_sock3.connect(("192.168.1.10", 98))
        mem_sock3.send(b'memory_block_02')
        mem_sock3.settimeout(0.5)
        self.mem_list.append(mem_sock3)
        time.sleep(0.01)

        #This is for the list scan values. The first is the memory values, the second is the buffer descriptor
        #In the memory, you chose the list of X, Y, etc. In the buffer descriptor, you need to change the number
        #of points you want to fetch, so the cycle is correctly performed.
        bd_sock_memory = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        bd_sock_memory.connect(("192.168.1.10", 98))
        bd_sock_memory.send(b'scan_list_engine')
        bd_sock_memory.settimeout(0.5)
        self.scan_memory_list = ScanListEngine(bd_sock_memory, 256 * 256)
        time.sleep(0.01)


        self.__dsp_filter = None
        self.__external_trigger = 0
        self.__flyback_pixels = None
        self.__x = None
        self.__y = None
        self.__pixel_ratio = None
        self.__rastering_mode = None

        self.dsp_filter = 0
        self.external_trigger = 0
        self.flyback_pixels = 0
        self.x_sampling = 128
        self.y_sampling = 128
        self.pixel_time = 1
        self.rastering_mode = 0

        self.update_acquisition_conditions()
        self.reset()

        self.frame = 0
        self.buffer_descriptor = 0
        self.counter = 0
        self.buffer_descriptor_length = 256 * 256 * 2
        self.max_frames = 0

        self.data_array = None

    def receive_total_frame(self, channel: int):
        if channel == 2:
            if self.get_dma_status() == 17:
                logging.info('***DMA***: DMA Internal Error.')
        #self.__lock.acquire()

        x = self.x_sampling
        y = self.y_sampling
        self.data_array = numpy.zeros(x * y, dtype='uint16')

        total_size = x * y * 2
        frame_size = 0
        frame = b''

        index = self.buffer_descriptor * self.buffer_descriptor_length + self.frame * total_size

        if channel == 3:
            temp_sock = self.scan_memory_list.sock
        else:
            temp_sock = self.mem_list[channel]

        message = message_creator(0, index, index + total_size, 0)
        temp_sock.send(message)

        while frame_size != total_size:
            next_bytes_size = min(total_size - frame_size, 1024)

            #message = message_creator(0, frame_size, frame_size + next_bytes_size, 0)
            #self.mem_list[channel].send(message)

            #try:
            temp = temp_sock.recv(next_bytes_size)

            #print(f"{frame_size}, {next_bytes_size}, {len(temp)}")
            #if len(temp) != 8192:
            #    print(len(temp))
            #except TimeoutError:
            #    #Retrying asking for the data
            #    print('bad')
            #    temp = b''
            #    self.mem_list[channel].send(message)
            frame = frame + temp
            frame_size = frame_size + len(temp)

        #self.__lock.release()

        self.data_array[:] = numpy.frombuffer(frame, dtype='uint16')
        return self.data_array.reshape((x, y))

    def update_frame(self):
        pass
        #self.frame = (self.frame + 1) % self.max_frames
        #if self.frame == self.max_frames:
        #self.counter += 1
        #self.buffer_descriptor = (self.buffer_descriptor + 1) % MAX_BUFFER_DESCRIPTORS

    def set_scan_register(self, initial_address: int, final_address: int, value: int) -> None:
        message = message_creator(1, initial_address, final_address, value)
        #self.__lock.acquire()
        send = self.scan_sock.send(message)
        time.sleep(0.01)
        #self.__lock.release()
        assert (send == 31)

    def set_dma_register(self, initial_address: int, final_address: int, value: int) -> None:
        message = message_creator(1, initial_address, final_address, value)
        #self.__lock.acquire()
        send = self.spim_sock.send(message)
        time.sleep(0.01)
        #self.__lock.release()
        assert (send == 31)

    def get_dma_status(self) -> int:
        message = message_creator(0, 0x34, 0x38, 0)
        #self.__lock.acquire()
        send = self.spim_sock.send(message)
        time.sleep(0.01)
        response = self.spim_sock.recv(512)
        response_decoded = int.from_bytes(response, "little")
        #self.__lock.release()
        assert (send == 31)
        return response_decoded


    def set_video_register(self, initial_address: int, final_address: int, value: int) -> None:
        message = message_creator(1, initial_address, final_address, value)
        #self.__lock.acquire()
        send = self.video_sock.send(message)
        time.sleep(0.01)
        #self.__lock.release()
        assert (send == 31)

    def set_video_filter(self, value):
        self.set_video_register(0, 4, value)

    def set_hyperspectral_ready(self):
        self.set_dma_register(88, 92, 0x100000)

    def update_acquisition_conditions(self):
        self.max_frames = int(256 * 256 * 2 / (2 * self.x_sampling * self.y_sampling))
        self.frame = 0
        self.buffer_descriptor = 0
        #self.set_scan_register(20, 24, int(256 * 256 * 2 / (2 * self.x_sampling * self.y_sampling)))
        self.set_scan_register(20, 24, 1)
        self.set_hyperspectral_ready()
        # self.data_array = numpy.zeros(self.x_sampling * self.y_sampling, dtype='uint16')

    def set_sampling(self, x, y):
        if x != self.x_sampling or y != self.y_sampling:
            self.scan_memory_list.create_square_list(x, y, self.__pixel_ratio, self.__external_trigger, self.__rastering_mode)
        self.x_sampling = x
        self.y_sampling = y

    def reset(self):
        self.set_scan_register(16, 20, 1)
        self.set_scan_register(16, 20, 0)

    # def set_pixel_time(self, pixel_time_us):
    #    nb_clock_ticks = int(pixel_time_us * 1000.0 / CLOCK_TICK)
    #    self.pixel_ratio = nb_clock_ticks
    #    self.set_scan_register(0, 4, nb_clock_ticks)
    #    self.reset()

    def data_array_reshaped(self):
        return self.data_array.reshape((self.x_sampling, self.y_sampling))

    @property
    def pixel_time(self):
        return self.__pixel_ratio

    @pixel_time.setter
    def pixel_time(self, value):
        nb_clock_ticks = int(value * 1000.0 / CLOCK_TICK)
        if self.__pixel_ratio != nb_clock_ticks:
            self.__pixel_ratio = nb_clock_ticks
            self.set_scan_register(0, 4, nb_clock_ticks)
            self.scan_memory_list.create_square_list(self.__x, self.__y, self.__pixel_ratio, self.__external_trigger,
                                                     self.__rastering_mode)

    @property
    def x_sampling(self):
        return self.__x

    @x_sampling.setter
    def x_sampling(self, value):
        if self.__x != value:
            self.__x = value
            self.set_scan_register(8, 12, value)

    @property
    def y_sampling(self):
        return self.__y

    @y_sampling.setter
    def y_sampling(self, value):
        if self.__y != value:
            self.__y = value
            self.set_scan_register(12, 16, value)

    @property
    def flyback_pixels(self):
        return self.__flyback_pixels

    @flyback_pixels.setter
    def flyback_pixels(self, value):
        if self.__flyback_pixels != value:
            self.__flyback_pixels = value
            self.set_scan_register(4, 8, value)

    @property
    def external_trigger(self):
        return self.__external_trigger

    @external_trigger.setter
    def external_trigger(self, value):
        if self.__external_trigger != value:
            self.__external_trigger = value
            self.set_scan_register(24, 28, value)
            self.scan_memory_list.create_square_list(self.__x, self.__y, self.__pixel_ratio, self.__external_trigger,
                                                     self.__rastering_mode)

    @property
    def dsp_filter(self):
        return self.__dsp_filter

    @dsp_filter.setter
    def dsp_filter(self, value):
        if self.__dsp_filter != value:
            self.__dsp_filter = value
            self.set_video_filter(self.__dsp_filter)

    @property
    def rastering_mode(self):
        return self.__rastering_mode

    @rastering_mode.setter
    def rastering_mode(self, value):
        if self.__rastering_mode != value:
            self.__rastering_mode = value
            self.scan_memory_list.create_square_list(self.__x, self.__y, self.__pixel_ratio, self.__external_trigger,
                                                     self.__rastering_mode)


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
        self.scan_engine.set_sampling(x, y)
        self.scan_engine.pixel_time = pixel_time
        self.scan_engine.update_acquisition_conditions()
        self.scan_engine.reset()

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

        self.scan_engine.update_frame()
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
        # This is a very strange behavior of geometry class. Very misleading. Value is a tuple like
        # (x=0.1, y=0.3) but value[0] gives y value while value[1] gives x value. You can check here
        # print(f'value is {value} and first is {value[0]}. Second is {value[1]}')
        # If you using this func, please call it with (y, x)
        px, py = round(self.__probe_position[1] * self.__scan_area[1]), round(
            self.__probe_position[0] * self.__scan_area[0])
        self.__probe_position_pixels = [px, py]

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
