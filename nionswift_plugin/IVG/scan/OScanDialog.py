from nion.ui import Declarative
from nion.utils import Event
from nion.swift.model import HardwareSource

class Handler:
    def __init__(self):
        self.property_changed_event = Event.Event()
        scan = HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id("open_scan_device")
        self.scan = scan.scan_device.scan_engine

    def set_video_filter(self, value):
        self.scan.set_video_filter(value)

    @property
    def flyback(self):
        return self.scan.flyback_pixels

    @flyback.setter
    def flyback(self, value):
        try:
            self.scan.flyback_pixels = int(value)
        except TypeError:
            pass

    @property
    def dsp_filter(self):
        return self.scan.dsp_filter

    @dsp_filter.setter
    def dsp_filter(self, value):
        try:
            self.scan.dsp_filter = value
        except TypeError:
            pass

    @property
    def rastering_mode(self):
        return self.scan.rastering_mode

    @rastering_mode.setter
    def rastering_mode(self, value):
        try:
            self.scan.rastering_mode = value
        except TypeError:
            pass

    @property
    def external_trigger(self):
        return self.scan.external_trigger

    @external_trigger.setter
    def external_trigger(self, value):
        try:
            self.scan.external_trigger = int(value)
        except TypeError:
            #When opening the GUI, CheckBoxWidget is passed as an argument. We ignore this value
            pass

    #def external_trigger_set(self, widget, checked):
    #    self.scan.external_trigger = int(checked)


class View():
    def __init__(self):
        ui = Declarative.DeclarativeUI()
        self.flyback_text = ui.create_label(name='flyback_text', text="Flyback value: ")
        self.flyback_value = ui.create_line_edit(name='flyback_value', text='@binding(flyback)')
        self.flyback_row = ui.create_row(self.flyback_text, self.flyback_value, ui.create_stretch())

        self.filter_text = ui.create_label(name='filter_text', text="DSP Filter: ")
        self.filter_value = ui.create_combo_box(items=['0', '1', '2', '4', '8', '16', '32'],
                                                current_index='@binding(dsp_filter)',
                                                name='filter_value', width = '100')
        self.filter_row = ui.create_row(self.filter_text, self.filter_value, ui.create_stretch())

        self.rastering_text = ui.create_label(name='rastering_text', text="Rastering mode: ")
        self.rastering_value = ui.create_combo_box(items=['Normal', 'Random', 'Serpentine'],
                                                current_index='@binding(rastering_mode)',
                                                name='rastering_value', width='100')
        self.rastering_row = ui.create_row(self.rastering_text, self.rastering_value, ui.create_stretch())

        self.external_trigger_text = ui.create_label(name='external_trigger_text', text="External Trigger: ")
        self.external_trigger = ui.create_check_box(name='external_trigger', checked = '@binding(external_trigger)')
        self.external_trigger_row = ui.create_row(self.external_trigger_text, self.external_trigger, ui.create_stretch())

        self.column = ui.create_column(self.flyback_row, self.filter_row, self.rastering_row, self.external_trigger_row)
        self.dialog = ui.create_modeless_dialog(self.column, title="Scan Settings")

class ConfigDialog:
    def __init__(self, document_controller):
        # document_controller_ui AND document_controller should be passed to the construct....
        self.widget = Declarative.construct(document_controller.ui, document_controller, View().dialog, Handler())
        if self.widget != None:
            self.widget.show()