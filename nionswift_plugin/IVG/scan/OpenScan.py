# standard libraries
import copy
import math
import gettext
import numpy
import threading
import typing
import time
import socket

# local libraries
from nion.utils import Registry
from nion.utils import Geometry
from nion.instrumentation import scan_base
from nion.instrumentation import stem_controller

from nionswift_plugin.IVG.scan.OpenScanConfigDialog import ConfigDialog

from nionswift_plugin.IVG import ivg_inst

_ = gettext.gettext

BUFFER_SIZE = 2048
PACKET_LEN = 1024
CLOCK_FREQUENCY = 80
CLOCK_TICK = 1 / CLOCK_FREQUENCY * 1e3 #in nanoseconds


class ScanEngine:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)
        self.sock.connect(("192.168.1.10", 7))
        self.x_sampling = 128
        self.y_sampling = 128
        self.current_frame = b''
        self.data_array = numpy.zeros(self.x_sampling * self.y_sampling, dtype='uint32')
        print('OK')

    def set_scan_dma(self):
        data = [0, 3, 0, 0, 0, 0]
        self.sock.sendall(bytearray(data))
        return self.sock.recv(4)

    def finish_scan_dma(self):
        data = [0, 4, 0, 0, 0, 0]
        self.sock.sendall(bytearray(data))
        return self.sock.recv(4)

    def query_bd_count(self):
        data = [0, 5, 0, 0, 0, 0]
        self.sock.sendall(bytearray(data))
        return self.sock.recv(4)

    def receive_frame(self):
        data = [0, 6, 0, 0, 0, 0]
        self.sock.sendall(bytearray(data))
        size = 0
        frame = b''
        #frame = self.sock.recv(self.x_sampling * self.y_sampling * 4)
        while size != self.x_sampling * self.y_sampling * 4:
            temp = self.sock.recv(self.x_sampling * self.y_sampling * 4 - size)
            size = size + len(temp)
            frame = frame + temp
        temp_data = numpy.frombuffer(frame, dtype='uint32')
        self.data_array = temp_data
        return None

    def receive_partial_frame(self, memory_offset, size):
        data = [0, 7, memory_offset >> 8, memory_offset % 256, size >> 8, size % 256]
        self.sock.sendall(bytearray(data))
        frame_size = 0
        frame = b''
        while frame_size != size:
            temp = self.sock.recv(size - frame_size)
            frame_size = frame_size + len(temp)
            frame = frame + temp
        temp_data = numpy.frombuffer(frame, dtype='uint32')
        self.data_array[memory_offset >> 2 : (memory_offset >> 2) + (size >> 2)] = temp_data
        return None

    def listen_from_tcp(self):
        print(self.x_sampling)
        print(self.y_sampling)
        try:
            a = self.sock.recv(self.x_sampling * self.y_sampling * 4)
            temp_data = numpy.frombuffer(a, dtype='uint32')
            print(temp_data)
            print(len(temp_data))
            #self.data_array = temp_data
        except:
            print('not more images')

        #
        return None

    def set_scan_register(self, address, value):
        data = [0, 1, address >> 8, address % 256, value >> 8, value % 256]
        self.sock.sendall(bytearray(data))
        return self.sock.recv(4)

    def set_video_register(self, address, value):
        data = [0, 2, address >> 8, address % 256, value >> 8, value % 256]
        self.sock.sendall(bytearray(data))
        return self.sock.recv(4)

    def set_video_filter(self, value):
        self.set_video_register(1, value)

    def set_sampling(self, x, y):
        self.x_sampling = x
        self.y_sampling = y
        self.set_scan_register(2, x)
        self.set_scan_register(3, y)
        self.reset()
        self.data_array = numpy.zeros(self.x_sampling * self.y_sampling, dtype='uint32')

    def reset(self):
        self.set_scan_register(4, 1)
        self.set_scan_register(4, 0)

    def set_pixel_time(self, pixel_time_us):
        nb_clock_ticks = int(pixel_time_us * 1000.0 / CLOCK_TICK)
        self.set_scan_register(0, nb_clock_ticks)
        self.reset()

    def set_array_data():
        return numpy.zeros(self.x_sampling * self.y_sampling, dtype='uint32')

    def data_array_reshaped(self):
        return self.data_array.reshape((self.x_sampling, self.y_sampling))

    def read_full_memory(self):
        for x in range(8):
            self.get_data_part_v2(8192 * x, 8192)

    def read_full_frame(self):
        max_columns = self.x_sampling * 4
        max_lines = self.y_sampling
        for x in range(max_lines):
            self.get_data_part_v2(512 * x, max_columns)

    def get_data_part_v2(self, memory_offset, size):
        data = [0, 0, memory_offset >> 8, memory_offset % 256, size >> 8, size % 256]
        self.sock.sendall(bytearray(data))
        line = memory_offset >> 9
        read_size = 0
        while read_size != int(size / 4):
            a = self.sock.recv(PACKET_LEN)
            temp_data = numpy.frombuffer(a, dtype='uint32')
            self.data_array[read_size + line * self.y_sampling: len(temp_data) + read_size + line * self.y_sampling] = temp_data
            read_size = read_size + len(temp_data)

    def get_data_part(self, memory_offset, size):
        data = [0, 0, memory_offset >> 8, memory_offset % 256, size >> 8, size % 256]
        self.sock.sendall(bytearray(data))
        read_size = 0
        while read_size != int(size / 4):
            a = self.sock.recv(PACKET_LEN)
            temp_data = numpy.frombuffer(a, dtype='uint32')
            self.data_array[
            read_size + int(memory_offset / 4): len(temp_data) + read_size + int(memory_offset / 4)] = temp_data
            read_size = read_size + len(temp_data)

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
        self.flyback_pixels = 2
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
        channels = [Channel(0, "ADF", True), Channel(1, "BF", False)]
        return channels

    def __get_initial_profiles(self) -> typing.List[scan_base.ScanFrameParameters]:
        profiles = list()
        profiles.append(scan_base.ScanFrameParameters({"size": (128, 128), "pixel_time_us": 0.5, "fov_nm": 4000., "rotation_rad": 0.393}))
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
        print(frame_parameters)
        print(frame_parameters.as_dict())
        (x, y) = frame_parameters.as_dict()['size']
        pixel_time = frame_parameters.as_dict()['pixel_time_us']
        self.scan_engine.set_sampling(x, y)
        self.scan_engine.set_pixel_time(pixel_time)

    def save_frame_parameters(self) -> None:
        """Called when shutting down. Save frame parameters to persistent storage."""
        pass

    def start_frame(self, is_continuous: bool) -> int:
        """Start acquiring. Return the frame number."""
        if not self.__is_scanning:
            self.__buffer = list()
            self.scan_engine.set_scan_dma()
            self.__start_next_frame()
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

        #gotit = self.has_data_event.wait(1.0)

        if self.__frame is None:
            self.__start_next_frame()

        current_frame = self.__frame  # this is from Frame Class defined above
        assert current_frame is not None
        data_elements = list()

        for channel in current_frame.channels:
            data_element = dict()

            self.scan_engine.query_bd_count()
            #self.scan_engine.receive_frame()
            total_size = self.scan_engine.x_sampling * self.scan_engine.y_sampling * 4
            frame_size = min(total_size, 32768)
            self.scan_engine.receive_partial_frame(0, frame_size)
            data_array = self.scan_engine.data_array_reshaped()
            data_element["data"] = data_array
            properties = current_frame.frame_parameters.as_dict()
            properties["center_x_nm"] = current_frame.frame_parameters.center_nm[1]
            properties["center_y_nm"] = current_frame.frame_parameters.center_nm[0]
            properties["rotation_deg"] = math.degrees(current_frame.frame_parameters.rotation_rad)
            properties["channel_id"] = channel.channel_id
            data_element["properties"] = properties
            if data_array is not None:
                data_elements.append(data_element)



        #self.has_data_event.clear()

        current_frame.complete = True
        if current_frame.complete:
            self.__frame = None


        # return data_elements, complete, bad_frame, sub_area, frame_number, pixels_to_skip
        return data_elements, current_frame.complete, False, ((0, 0), data_array.shape), None, 0

    #This one is called in scan_base
    def prepare_synchronized_scan(self, scan_frame_parameters: scan_base.ScanFrameParameters, *, camera_exposure_ms, **kwargs) -> None:
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
        self.__rotation = value*180/numpy.pi

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
        #This is a very strange behavior of geometry class. Very misleading. Value is a tuple like
        #(x=0.1, y=0.3) but value[0] gives y value while value[1] gives x value. You can check here
        #print(f'value is {value} and first is {value[0]}. Second is {value[1]}')
        #If you using this func, please call it with (y, x)
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
        api = api_broker.get_api(version="1", ui_version="1")
        document_controller = api.application.document_controllers[0]._document_controller
        myConfig = ConfigDialog(document_controller)


def run(instrument: ivg_inst.ivgInstrument):
    scan_device = Device(instrument)
    component_types = {"scan_device"}  # the set of component types that this component represents
    Registry.register_component(scan_device, component_types)