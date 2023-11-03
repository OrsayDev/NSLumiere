from nion.ui import Declarative
from nion.utils import Event
from nion.swift.model import HardwareSource

ACQUISITION_WINDOW = ['boxcar', 'triang', 'blackman', 'hamming', 'hann']

class Handler:
    def __init__(self):
        self.property_changed_event = Event.Event()
        scan = HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id("open_scan_device")
        self.scan = scan.scan_device.scan_engine

    @property
    def imagedisplay(self):
        return self.scan.imagedisplay

    @imagedisplay.setter
    def imagedisplay(self, value):
        self.scan.imagedisplay = value
    @property
    def flyback_us(self):
        return self.scan.flyback_us

    @flyback_us.setter
    def flyback_us(self, value):
        self.scan.flyback_us = value

    @property
    def kernel_mode(self):
        return self.scan.kernel_mode

    @kernel_mode.setter
    def kernel_mode(self, value):
        self.scan.kernel_mode = value

    @property
    def dsp_filter(self):
        return self.scan.dsp_filter

    @dsp_filter.setter
    def dsp_filter(self, value):
        self.scan.dsp_filter = value

    @property
    def adc_acquisition_mode(self):
        return self.scan.adc_acquisition_mode

    @adc_acquisition_mode.setter
    def adc_acquisition_mode(self, value):
        self.scan.adc_acquisition_mode = value

    @property
    def acquisition_cutoff(self):
        return self.scan.acquisition_cutoff

    @acquisition_cutoff.setter
    def acquisition_cutoff(self, value):
        self.scan.acquisition_cutoff = value

    @property
    def acquisition_window(self):
        return self.scan.acquisition_window

    @acquisition_window.setter
    def acquisition_window(self, value):
        self.scan.acquisition_window = value

    @property
    def rastering_mode(self):
        return self.scan.rastering_mode

    @rastering_mode.setter
    def rastering_mode(self, value):
        self.scan.rastering_mode = value

    @property
    def lissajous_nx(self):
        return self.scan.lissajous_nx

    @lissajous_nx.setter
    def lissajous_nx(self, value):
        self.scan.lissajous_nx = value

    @property
    def lissajous_ny(self):
        return self.scan.lissajous_ny

    @lissajous_ny.setter
    def lissajous_ny(self, value):
        self.scan.lissajous_ny = value

    @property
    def lissajous_phase(self):
        return self.scan.lissajous_phase

    @lissajous_phase.setter
    def lissajous_phase(self, value):
        self.scan.lissajous_phase = value

    @property
    def external_trigger(self):
        return self.scan.external_trigger

    @external_trigger.setter
    def external_trigger(self, value):
        if value == True or value == False:
            self.scan.external_trigger = int(value)

    @property
    def magswitches(self):
        return self.scan.magboard_switches

    @magswitches.setter
    def magswitches(self, value):
        self.scan.magboard_switches = value

    @property
    def offset_adc(self):
        return self.scan.offset_adc

    @offset_adc.setter
    def offset_adc(self, value):
        self.scan.offset_adc = value

class View():
    def __init__(self):
        ui = Declarative.DeclarativeUI()
        #Acquisition parameters
        self.imageshow_text = ui.create_label(name='imageshow_text', text="Image display: ")
        self.imageshow_value = ui.create_combo_box(
            items=['Normal', 'Ordered', 'DAC-based'],
            current_index='@binding(imagedisplay)',
            name='rastering_value', width='100')
        self.imageshow_row = ui.create_row(self.imageshow_text, self.imageshow_value, ui.create_stretch())

        self.acquisition_parameters_group = ui.create_group(title='Acquisition parameters', content=ui.create_column(
            self.imageshow_row
        ))

        #Rastering parameters
        self.flyback_text = ui.create_label(name='flyback_text', text="Flyback value (us): ")
        self.flyback_value = ui.create_line_edit(name='flyback_value', text='@binding(flyback_us)')
        self.flyback_row = ui.create_row(self.flyback_text, self.flyback_value, ui.create_stretch())

        self.external_trigger_text = ui.create_label(name='external_trigger_text', text="External Trigger: ")
        self.external_trigger = ui.create_check_box(name='external_trigger', checked='@binding(external_trigger)')
        self.external_trigger_row = ui.create_row(self.external_trigger_text, self.external_trigger,
                                                  ui.create_stretch())

        self.rastering_text = ui.create_label(name='rastering_text', text="Rastering mode: ")
        self.rastering_value = ui.create_combo_box(items=['Normal', 'Serpentine', 'Random', 'Lissajous', 'Saw lissajous'],
                                                   current_index='@binding(rastering_mode)',
                                                   name='rastering_value', width='100')
        self.rastering_row = ui.create_row(self.rastering_text, self.rastering_value, ui.create_stretch())

        self.lissajous_nx_text = ui.create_label(name='lissajous_text', text="Liss. nx: ")
        self.lissajous_nx_value = ui.create_line_edit(name='lissajous_value: ', width=45, text='@binding(lissajous_nx)')
        self.lissajous_ny_text = ui.create_label(name='lissajous_text', text="Liss. ny: ")
        self.lissajous_ny_value = ui.create_line_edit(name='lissajous_value: ', width=45, text='@binding(lissajous_ny)')
        self.lissajous_phase_text = ui.create_label(name='lissajous_phase_text', text="Lissajous phase: ")
        self.lissajous_phase_value = ui.create_line_edit(name='lissajous_phase_value: ', width=30,
                                                         text='@binding(lissajous_phase)')
        self.lissajous_row = ui.create_row(self.lissajous_nx_text, self.lissajous_nx_value, ui.create_spacing(5),
                                           self.lissajous_ny_text, self.lissajous_ny_value, ui.create_spacing(5),
                                           self.lissajous_phase_text, self.lissajous_phase_value,
                                           ui.create_stretch())

        self.rastering_group = ui.create_group(title='Scanning parameters', content=ui.create_column(
            self.flyback_row, self.rastering_row, self.lissajous_row
        ))
        #ADC parameters

        self.kernel_mode_text = ui.create_label(name='kernel_mode_text', text="DSP Kernel: ")
        self.kernel_mode_value = ui.create_combo_box(items=['First pixel', 'Custom', 'Square', 'Triangular', 'Gaussian'],
                                                current_index='@binding(kernel_mode)',
                                                name='kernel_mode_value', width='100')
        self.kernel_mode_row = ui.create_row(self.kernel_mode_text, self.kernel_mode_value, ui.create_stretch())

        self.filter_text = ui.create_label(name='filter_text', text="IIR Filter: ")
        self.filter_value = ui.create_combo_box(items=['1', '2', '4', '8', '16', '32', '64'],
                                                current_index='@binding(dsp_filter)',
                                                name='filter_value', width = '100')
        self.filter_row = ui.create_row(self.filter_text, self.filter_value, ui.create_stretch())

        self.acqusition_adc_text = ui.create_label(name='acqusition_adc_text', text="ADC type: ")
        self.acquisition_adc_value = ui.create_combo_box(items=['FIR', 'Pixel counter', 'IIR', 'Multip.', 'Kernel'],
                                                current_index='@binding(adc_acquisition_mode)',
                                                name='acquisition_adc_value', width='100')

        self.acquisition_adc_row = ui.create_row(self.acqusition_adc_text, self.acquisition_adc_value, ui.create_stretch())

        self.cutoff_text = ui.create_label(name='cutoff_text', text='Cutoff (KHz): ')
        self.cutoff_value = ui.create_line_edit(name='cutoff_value', text='@binding(acquisition_cutoff)')

        self.window_text = ui.create_label(name='cutoff_text', text='Window: ')
        self.window_value = ui.create_combo_box(name='window_value',
                                                items=ACQUISITION_WINDOW,
                                                current_index='@binding(acquisition_window)')
        self.adc_filter_row = ui.create_row(self.cutoff_text, self.cutoff_value, ui.create_stretch(),
                                            self.window_text, self.window_value)

        self.adc_parameters_group = ui.create_group(title='ADC parameters', content=ui.create_column(
            self.filter_row, self.acquisition_adc_row, self.kernel_mode_row, self.adc_filter_row
        ))

        #Hardware settings
        self.magswitches_text = ui.create_label(name='magswitches_text', text="Mag. Switches: ")
        self.magswitches_value = ui.create_line_edit(name='magswitches_value: ', text='@binding(magswitches)')
        self.magswitches_row = ui.create_row(self.magswitches_text, self.magswitches_value,
                                                  ui.create_stretch())

        self.offset_adc_text = ui.create_label(name='offset_adc_text', text="Off. ADC: ")
        self.offset_adc_value = ui.create_line_edit(name='offset_adc_value: ', text='@binding(offset_adc)')
        self.offset_adc_row = ui.create_row(self.offset_adc_text, self.offset_adc_value,
                                             ui.create_stretch())

        self.hardware_settings_group = ui.create_group(title='Hardware settings', content=ui.create_column(
            self.magswitches_row, self.offset_adc_row
        ))

        #Creating final GUI
        self.column = ui.create_column(self.acquisition_parameters_group, self.rastering_group, self.adc_parameters_group, self.hardware_settings_group)
        self.dialog = ui.create_modeless_dialog(self.column, title="Scan Settings")

class ConfigDialog:
    def __init__(self, document_controller):
        # document_controller_ui AND document_controller should be passed to the construct....
        self.widget = Declarative.construct(document_controller.ui, document_controller, View().dialog, Handler())
        if self.widget != None:
            self.widget.show()