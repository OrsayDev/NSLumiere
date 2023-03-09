from nion.ui import Declarative
from nion.utils import Event
import socket

class Handler:
    def __init__(self):
        self.property_changed_event = Event.Event()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(1.0)
        self.sock.connect(("192.168.1.10", 7))
        self.__dsp_filter = 0

    def set_video_register(self, address, value):
        data = [0, 2, value % 256, value >> 8, address % 256, address >> 8]
        self.sock.sendall(bytearray(data))
        return self.sock.recv(4)

    def set_video_filter(self, value):
        self.set_video_register(1, value)

    @property
    def dsp_filter(self):
        return self.__dsp_filter

    @dsp_filter.setter
    def dsp_filter(self, value):
        self.__dsp_filter = value
        self.set_video_filter(value)


class View():
    def __init__(self):
        ui = Declarative.DeclarativeUI()
        self.filter_text = ui.create_label(name='filter_text', text="DSP Filter: ")
        self.filter_value = ui.create_combo_box(items=['0', '1', '2', '4', '8', '16', '32'],
                                                current_index='@binding(dsp_filter)',
                                                name='filter_value', width = '100')
        self.filter_row = ui.create_row(self.filter_text, self.filter_value, ui.create_stretch())

        self.column = ui.create_column(self.filter_row)
        self.dialog = ui.create_modeless_dialog(self.column, title="Scan Settings")

class ConfigDialog:
    def __init__(self, document_controller):
        # document_controller_ui AND document_controller should be passed to the construct....
        self.widget = Declarative.construct(document_controller.ui, document_controller, View().dialog, Handler())
        if self.widget != None:
            self.widget.show()
