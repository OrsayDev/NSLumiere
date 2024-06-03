import gettext
import logging

from nion.utils import Converter
from nion.utils import Model
from nion.utils import Registry

from nionswift_plugin.IVG.camera import VGCameraYves

_ = gettext.gettext


class CameraHandler:
    """Handle interaction between the user interface and the hardware.

    This class should respond to changes from the UI by controlling the hardware. Conversely, it should respond to
    changes from the hardware by updating the UI. Some care should be given to avoiding UI loops.
    """

    # these variables will be filled in when the UI view is constructed.
    cancel_button = None
    stop_button = None
    start_button = None

    # models used

    def __init__(self, api, event_loop, ui_view, hardware_source_id, camera_device: VGCameraYves.CameraDevice,
                 camera_settings: VGCameraYves.CameraSettings):
        self.event_loop = event_loop
        self.ui_view = ui_view

        self.hardware_source = api.get_hardware_source_by_id(hardware_source_id, "~1.0")
        self.__hardware_source = self.hardware_source._hardware_source
        self.camera_device = camera_device
        self.camera_settings = camera_settings

        def frame_parameter_changed(name, *args, **kwargs):
            pass
            #self.event_loop.create_task(self.update_buttons())
            #self.stop_clicked()

        self.__frame_parameter_changed_event_listener = camera_device.frame_parameter_changed_event.listen(frame_parameter_changed)
        self.__stop_acquisition_event_listener = camera_device.stop_acquitisition_event.listen(self.stop_clicked)
        self.__current_event_listener = camera_device.current_event.listen(self.update_current)

        sx, sy = camera_device.camera.getCCDSize()

        if camera_device.isMedipix:
            self.__areas = [(0, 0, sy, sx), (sy / 2, 0, sy, sx), (3 * sy / 4, 0, sy, sx), (7 * sy / 8, 0, sy, sx),
                            (15 * sy / 16, 0, sy, sx)]
            self.__areas_names = [_("256"), _("128"), _("64"), _("32"), _("16")]
            self.h_binning_values = [1, 2, 4, 8, 16]
            self.v_binning_values = [1, sy]
        else:
            self.__areas = [(0, 0, sy, sx), (sy / 4, 0, 3 * sy / 4, sx), (3 * sy / 8, 0, 5 * sy / 8, sx),(sy / 2 - 5, 0, sy / 2 + 5, sx)]
            self.__areas_names = [_("Full"), _("Half"), _("Quater"), _("Skinny")]
            self.h_binning_values = [1, 2, 4, 8, 16]

            if sy == 200:
                self.v_binning_values = [1, 2, 5, 10, 20, 50, 100, 200]
            elif sy == 100:
                self.v_binning_values = [1, 2, 5, 10, 20, 50, 100]
            elif sy == 256:
                self.v_binning_values = [1, 2, 4, 8, 16, 32, 64, 128, 256]
            else:
                self.v_binning_values = [1]

        frame_parameters = camera_settings.get_current_frame_parameters()

        area_enum = 0
        correction_enum = 0
        self.roi_items = self.__areas_names
        self.roi_item = Model.PropertyModel(area_enum)
        self.roi_item_text = Model.PropertyModel("???r")
        self.port_items = Model.PropertyModel([])
        self.port_items.value = list(self.camera_device.camera.getPortNames())
        self.port_item = Model.PropertyModel(frame_parameters["port"])
        self.port_item_text = Model.PropertyModel("")
        self.speed_items = Model.PropertyModel([])
        self.speed_items.value = list(self.camera_device.camera.getSpeeds(frame_parameters["port"]))
        self.speed_item = Model.PropertyModel(-1)
        self.speed_item_text = Model.PropertyModel("???s")
        self.chips_items = Model.PropertyModel([])
        self.chips_items.value = list(["All", "1", "2", "3", "4"])
        self.chips_item = Model.PropertyModel(0)
        self.chips_item_text = Model.PropertyModel("???s")
        self.flip_model = Model.PropertyModel(frame_parameters["flipped"])
        self.current_value = Model.PropertyModel("0")
        self.delay_value = Model.PropertyModel(frame_parameters["timeDelay"])
        self.width_value = Model.PropertyModel(frame_parameters["timeWidth"])
        self.time_converter = Converter.PhysicalValueToStringConverter("(1.5625 ns)", 1)
        self.gain_items = Model.PropertyModel([])
        self.gain_items.value = list(self.camera_device.camera.getGains(frame_parameters["port"]))
        self.gain_item = Model.PropertyModel(frame_parameters["gain"])
        self.gain_item_text = Model.PropertyModel("???g")
        self.multiplication_converter = Converter.IntegerToStringConverter()
        self.multiplication_model = Model.PropertyModel(frame_parameters["multiplication"])
        self.h_binning_item = Model.PropertyModel(frame_parameters["h_binning"])
        self.v_binning_items = Model.PropertyModel([])
        self.v_binning_items.value = list(self.v_binning_values, )
        self.v_binning_item = Model.PropertyModel(frame_parameters["v_binning"])
        self.v_binning_item_text = Model.PropertyModel("???s")
        self.exposure_converter = Converter.FloatToStringConverter(format="{0:.2f}")
        self.exposure_model = Model.PropertyModel(frame_parameters["exposure_ms"])
        self.threshold_model = Model.PropertyModel(frame_parameters["video_threshold"])
        self.threshold_converter = Converter.IntegerToStringConverter()
        self.nbspectra_model = Model.PropertyModel(frame_parameters["spectra_count"])
        self.nbspectra_converter = Converter.IntegerToStringConverter()
        self.tab_v_binning = Model.PropertyModel(0)
        self.status_text = Model.PropertyModel("Stopped")
        correction_enum = 0
        if self.camera_device.isMedipix or self.camera_device.isTimepix:
            self.correction_items = [_("No gaps"), _("Gaps zero"), _("Gaps filled")]
        else:
            self.correction_items = [_("None"), _("Readout"), _("Gain"), _("Both")]
        self.correction_item = Model.PropertyModel(correction_enum)

        self.soft_binning_model = Model.PropertyModel(frame_parameters["soft_binning"])

        self.mode_items = Model.PropertyModel([])
        self.mode_items.value = self.camera_settings.modes
        val = self.camera_settings.modes.index(frame_parameters["acquisition_mode"])
        self.mode_item = Model.PropertyModel(self.camera_settings.modes.index(frame_parameters["acquisition_mode"]))
        self.mode_item_text = Model.PropertyModel("???g")

        self.synchro_items = Model.PropertyModel([])
        self.synchro_items.value = self.camera_settings.synchro_modes
        val = self.camera_settings.synchro_modes.index(frame_parameters["synchro_mode"])
        self.synchro_item = Model.PropertyModel(
            self.camera_settings.synchro_modes.index(frame_parameters["synchro_mode"]))
        self.synchro_item_text = Model.PropertyModel("???g")

        self.tp3mode_item = Model.PropertyModel(frame_parameters["tp3mode"])
        if self.camera_device.isTimepix:
            self.tp3mode_items = list(self.camera_device.camera.getTp3Modes())
        else:
            self.tp3mode_items = list(['None'])

        self.tp3_bias_voltage = Model.PropertyModel(frame_parameters["bias_V"])
        self.tp3_time_resolved = Model.PropertyModel(frame_parameters["time_resolved"])
        #def frame_parameter_changed(name, *args, **kwargs):
        #    if name == "acquisition_mode":
        #        md = self.camera_settings.get_current_frame_parameters()["acquisition_mode"]
        #        md1 = self.camera_settings.modes.index(md) if md in self.camera_settings.modes else 0
        #        self.mode_item.value = md1

        #self.__frame_parameter_changed_event_listener = camera_device.frame_parameter_changed_event.listen(
        #    frame_parameter_changed)

        # self.shutter_enabled_model = Model.PropertyModel(False)
        self.fan_enabled_model = Model.PropertyModel(True)

        self.__acquisition_state_changed_event_listener = self.__hardware_source.acquisition_state_changed_event.listen(
            self.__acquisition_state_changed)

    def init_handler(self):
        """Initialize the UI after it has been constructed."""

        def update_speeds():
            try:
                self.speed_items.value = list(self.camera_device.camera.getSpeeds(self.port_item.value))
                self.speed_item.value = self.camera_settings.get_current_frame_parameters()["speed"]
                self.speed_item_text.value = self.speed_items.value[self.speed_item.value]
            except:
                self.speed_item.value = 0
                self.speed_item_text.value = self.speed_items.value[self.speed_item.value]
                logging.info('***CAMERA***: The new port values does not support current speed. Please recheck indexes.')

        def update_chips():
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            self.chips_item.value = [15,1,2,4,8].index(frame_parameters["chips_config"])
            self.chips_item_text.value = self.chips_items.value[self.chips_item.value]

        def update_gaps():
            self.correction_item.value = self.camera_settings.get_current_frame_parameters()["gaps_mode"]

        def update_gains():
            self.gain_items.value = list(self.camera_device.camera.getGains(self.port_item.value))
            gain = self.camera_settings.get_current_frame_parameters()["gain"]
            self.gain_item.value = gain - 1

        def update_multiplier():
            if self.port_items.value[self.port_item.value] == "Electron Multiplied":
                self.multiplication_model.value = self.camera_settings.get_current_frame_parameters()["multiplication"]

        def update_threshold():
            self.threshold_model.value = self.camera_settings.get_current_frame_parameters()["video_threshold"]

        def update_all_setup_widgets():
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            area_enum = 0
            index = 0

            for area in self.__areas:
                if all(i == j for i, j in zip(area, frame_parameters["area"])):
                    area_enum = index
                index = index + 1
            self.roi_item.value = area_enum

            self.port_items.value = list(self.camera_device.camera.getPortNames())
            self.port_item.value = frame_parameters["port"]
            self.fan_enabled_model.value = frame_parameters["fan_enabled"]
            update_speeds()
            update_chips()
            update_gaps()
            update_gains()
            update_binning()
            update_multiplier()
            update_threshold()

        def update_binning():
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            bx, by = frame_parameters["h_binning"], frame_parameters["v_binning"]
            self.h_binning_item.value = self.h_binning_values.index(bx) if bx in self.h_binning_values else None
            if self.camera_device.isMedipix or self.camera_device.isKURO:
                area = frame_parameters["area"]
                self.v_binning_items.value = [1, area[2] - area[0]]
                if by == 1:
                    self.v_binning_item.value = 0
                else:
                    self.v_binning_item.value = 1
                    set_v_binning(area[2] - area[0])
            else:
                self.v_binning_item.value = self.v_binning_values.index(by) if by in self.v_binning_values else None

        def update_exposure():
            self.exposure_model.value = self.camera_settings.get_current_frame_parameters().exposure_ms

        def update_mode():
            md = self.camera_settings.get_current_frame_parameters()["acquisition_mode"]
            md1 = self.camera_settings.modes.index(md) if md in self.camera_settings.modes else 0
            self.mode_item.value = md1
            self.nbspectra_model.value = self.camera_settings.get_current_frame_parameters()["spectra_count"]

        def update_flip():
            self.flip_model.value=self.camera_settings.get_current_frame_parameters()["flipped"]

        def set_port(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["port"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)
            update_flip()
            update_speeds()
            update_gains()
            update_multiplier()

        def set_speed(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["speed"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_chips(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            if self.camera_device.isMedipix:
                values = list([15,1,2,4,8])
                frame_parameters["chips_config"] = values[value]
                area = list(frame_parameters["area"])
                if value == 0:
                    area[3] = 1024
                else:
                    area[3] = 256
                frame_parameters["area"] = tuple(area)
                self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_gaps(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["gaps_mode"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_gain(value):
            # print(f"Panel: set_gain {value}")
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["gain"] = value + 1
            self.camera_settings.set_current_frame_parameters(frame_parameters)
            self.gain_item_text.value = self.gain_items.value[value]

        def set_multiplier(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["multiplication"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_fan(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["fan_enabled"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_h_binning(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["h_binning"] = self.h_binning_values[value]
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_v_binning(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            # frame_parameters["v_binning"] = self.v_binning_values[value]
            frame_parameters["v_binning"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_v_binning_tab(value):
            print(f"Tab v binning changed to {value}")
            set_v_binning(self.v_binning_items.value[value])

        def set_exposure(value):
            # print("Panel: set_exposure")
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["exposure_ms"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_mode(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["acquisition_mode"] = self.mode_items.value[value]
            # update other param accordingly
            if "1D" in frame_parameters["acquisition_mode"]:
                # frame_parameters["acquisition_style"] = "1d"
                if self.camera_device.isMedipix or self.camera_device.isKURO:
                    self.v_binning_item.value = 1
                else:
                    self.soft_binning_model.value = True
            elif "2D" in frame_parameters["acquisition_mode"]:
                # frame_parameters["acquisition_style"] = "2d"
                if self.camera_device.isMedipix or self.camera_device.isKURO:
                    self.v_binning_item.value = 0
                else:
                    self.soft_binning_model.value = False
            self.camera_settings.set_current_frame_parameters(frame_parameters)
            self.mode_item_text.value = self.mode_items.value[value]

        def set_synchro(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["synchro_mode"] = self.synchro_items.value[value]
            self.camera_settings.set_current_frame_parameters(frame_parameters)
            self.synchro_item_text.value = self.synchro_items.value[value]

        def set_nbspectra(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["spectra_count"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_roi(value):
            area = list(self.__areas[value])
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            if self.camera_device.isMedipix and frame_parameters["chips_config"] == 15:
                area[3] = 1024
            else:
                area[3] = 256
            frame_parameters["area"] = tuple(area)
            self.camera_settings.set_current_frame_parameters(frame_parameters)
            if self.camera_device.isMedipix or self.camera_device.isKURO:
                self.v_binning_values = [1, area[2] - area[0]]
                update_binning()

        def set_soft_binning(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["soft_binning"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_flip(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["flipped"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_threshold(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["video_threshold"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_tp3_delay(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["timeDelay"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_tp3_width(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["timeWidth"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_tp3mode(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["tp3mode"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_tp3_bias(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["bias_V"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        def set_tp3_time_resolved(value):
            frame_parameters = self.camera_settings.get_current_frame_parameters()
            frame_parameters["time_resolved"] = value
            self.camera_settings.set_current_frame_parameters(frame_parameters)

        self.roi_item.on_value_changed = set_roi
        self.port_item.on_value_changed = set_port
        self.speed_item.on_value_changed = set_speed
        self.chips_item.on_value_changed = set_chips
        self.correction_item.on_value_changed = set_gaps
        self.gain_item.on_value_changed = set_gain
        self.multiplication_model.on_value_changed = set_multiplier
        self.threshold_model.value = 0
        self.threshold_model.on_value_changed = set_threshold
        self.fan_enabled_model.on_value_changed = set_fan

        self.h_binning_item.on_value_changed = set_h_binning
        self.v_binning_item.on_value_changed = set_v_binning_tab
        #self.tab_v_binning.on_value_changed = set_v_binning_tab
        self.exposure_model.on_value_changed = set_exposure
        self.nbspectra_model.on_value_changed = set_nbspectra
        self.soft_binning_model.on_value_changed = set_soft_binning
        self.flip_model.on_value_changed=set_flip
        if self.camera_device.isTimepix:
            self.delay_value.on_value_changed=set_tp3_delay
            self.width_value.on_value_changed=set_tp3_width
            self.tp3mode_item.on_value_changed = set_tp3mode
            self.tp3_bias_voltage.on_value_changed = set_tp3_bias
            self.tp3_time_resolved.on_value_changed=set_tp3_time_resolved

        self.mode_item.on_value_changed = set_mode
        self.synchro_item.on_value_changed = set_synchro

        #All the binding are done. You can set values here to force parameter initialization
        #For ex: self.exposure_model.value = 10.0
        if self.camera_device.isTimepix:
            self.tp3_time_resolved.value = False #Forced to NOT Time-resolved
            self.port_item.value = 4 #Forced to frame-based


        update_all_setup_widgets()
        update_binning()
        update_exposure()
        update_mode()
        self.camera_device.set_frame_parameters(self.camera_settings.get_current_frame_parameters())

        self.event_loop.create_task(self.update_buttons())

    def update_current(self, value):
        self.current_value.value = value

    def cancel_clicked(self, widget):
        """Handle cancel button click.

        This method is called when the user clicks on the button. The name of this method corresponds to the
        `on_clicked` property of the push button."""
        self.hardware_source.abort_playing()

    def stop_clicked(self, widget=None):
        """Handle stop button click.

        This method is called when the user clicks on the button. The name of this method corresponds to the
        `on_clicked` property of the push button."""
        self.hardware_source.stop_playing()

    def start_clicked(self, widget):
        """Handle start button click.

        This method is called when the user clicks on the button. The name of this method corresponds to the
        `on_clicked` property of the push button."""
        # print(f"At start exposure = {self.exposure_model.value}")
        self.hardware_source.start_playing()

    def measure_clicked(selfself, widget):
        pass

    async def update_buttons(self):
        is_playing = self.hardware_source.is_playing
        self.cancel_button.enabled = is_playing
        self.stop_button.enabled = is_playing
        self.start_button.enabled = not is_playing
        self.status_text.value = "Running" if is_playing else "Stopped"

    def __acquisition_state_changed(self, is_acquiring):
        # this message will come from a thread. use event loop to call functions on the main thread.
        self.event_loop.create_task(self.update_buttons())


class CameraPanelFactory:
    camera_panel_type = "orsay_camera_panel"

    def get_ui_handler(self, api_broker=None, event_loop=None, hardware_source_id=None,
                       camera_device=None, camera_settings=None, **kwargs):
        ui_view = self.__create_ui_view(api_broker.get_ui("~1.0"), camera_device)
        api = api_broker.get_api("~1.0")
        return CameraHandler(api, event_loop, ui_view, hardware_source_id, camera_device, camera_settings)

    def __create_ui_view(self, ui, camera_device) -> dict:
        """Creates the UI view by using the declarative UI library.

        The UI view is simply a Python dictionary, constructed with the help of the declarative UI library."""
        h_binning_row = ui.create_row(ui.create_label(text=_("Horiz.:")),
                                      ui.create_combo_box(items_ref="h_binning_values",
                                                          current_index="@binding(h_binning_item.value)"), spacing=2)
        v_binning_row = ui.create_row(ui.create_label(text=_("V:")),
                                      ui.create_combo_box(items_ref="v_binning_values",
                                                          current_index="@binding(v_binning_item.value)"), spacing=2)

        binning_row = ui.create_row(h_binning_row, v_binning_row, spacing=8)

        if camera_device.isMedipix:
            binning_group = ui.create_group(binning_row, title=_("Binning"))
        else:
            eels_row = ui.create_row(ui.create_label(text="0.63 eV/ch"),
                                     ui.create_check_box(text=_("Spectra"),
                                                         checked="@binding(soft_binning_model.value)"))

            binning_content = ui.create_column(binning_row, eels_row, spacing=8)

            binning_group = ui.create_group(binning_content, title=_("Binning"))

        task_combo_box = ui.create_combo_box(items_ref="@binding(mode_items.value)",
                                             current_index="@binding(mode_item.value)")

        nbspectra_line_edit = ui.create_line_edit(text="@binding(nbspectra_model.value, converter=nbspectra_converter)")

        experiment_row1 = ui.create_row(task_combo_box, nbspectra_line_edit, spacing=8)

        exposure_line_edit = ui.create_line_edit(text="@binding(exposure_model.value, converter=exposure_converter)")

        experiment_row2 = ui.create_row(ui.create_label(text=_("Dwell Time (ms)")), exposure_line_edit, spacing=8)

        experiment_content = ui.create_column(experiment_row1, experiment_row2, spacing=8)

        experiment_group = ui.create_group(experiment_content, title=_("Experiment"))

        mode_row = ui.create_row(ui.create_label(text=_("Mode")),
                                 ui.create_combo_box(items=[_("Standalone"), _("Master"), _("Slave")]))

        cancel_button = ui.create_push_button(name="cancel_button", text=_("Cancel"), on_clicked="cancel_clicked")
        stop_button = ui.create_push_button(name="stop_button", text=_("Stop"), on_clicked="stop_clicked")
        start_button = ui.create_push_button(name="start_button", text=_("Start"), on_clicked="start_clicked")

        buttons = ui.create_row(ui.create_stretch(), cancel_button, stop_button, start_button, spacing=8)

        tp3_time_resolved = ui.create_check_box(text='TR', checked='@binding(tp3_time_resolved.value)')
        current_label = ui.create_label(text='Current (pA): ')
        current_val = ui.create_label(text="@binding(current_value.value)")
        current_column = ui.create_row(current_label, current_val, ui.create_stretch(), tp3_time_resolved)

        delay_label = ui.create_label(text='Time Delay: ')
        delay_value = ui.create_line_edit(name='delay_label_value', text="@binding(delay_value.value, converter=time_converter)")
        delay_row = ui.create_row(delay_label, delay_value, ui.create_stretch())

        width_label = ui.create_label(text='Time Width: ')
        width_value = ui.create_line_edit(name='width_label_value', text="@binding(width_value.value, converter=time_converter)")
        width_row = ui.create_row(width_label, width_value, ui.create_stretch())

        tp3mode = ui.create_row(ui.create_label(text=_("Tp3 Mode: ")),
                              ui.create_combo_box(items_ref="@binding(tp3mode_items)",
                                                  current_index="@binding(tp3mode_item.value)", width=100), ui.create_stretch(),
                                ui.create_label(text=_("Tp3 Bias (V): ")),
                                ui.create_line_edit(text="@binding(tp3_bias_voltage.value)", width=75),
                                )

        tp3_column = ui.create_column(current_column, delay_row, width_row, tp3mode, spacing=2)
        tp3_group = ui.create_group(tp3_column, title=_("TimePix3"))

        """
        If Timepix, i've added a small supplementary setting
        """
        if camera_device.isTimepix:
            control_column = ui.create_column(binning_group, experiment_group, mode_row, buttons, tp3_group,
                                              ui.create_stretch(), spacing=8, margin=4)
        else:
            control_column = ui.create_column(binning_group, experiment_group, mode_row, buttons,
                                              ui.create_stretch(), spacing=8, margin=4)


        ports = camera_device.camera.getPortNames()

        if len(ports) > 1:
            if camera_device.isMedipix:
                status_row = ui.create_row(ui.create_label(text="@binding(port_item_text.value)"),
                                           ui.create_label(text="@binding(chips_item_text.value)"),
                                           ui.create_label(text="@binding(threshold_model.value)"),
                                           ui.create_label(text="@binding(gain_item_text.value)"),
                                           ui.create_label(text="@binding(status_text.value)"), spacing=8)
            else:
                status_row = ui.create_row(ui.create_label(text="@binding(port_item_text.value)"),
                                           ui.create_label(text="@binding(speed_item_text.value)"),
                                           ui.create_label(text="@binding(gain_item_text.value)"),
                                           ui.create_label(text="@binding(status_text.value)"), spacing=8)
        else:
            status_row = ui.create_row(ui.create_label(text="@binding(speed_item_text.value)"),
                                       ui.create_label(text="@binding(gain_item_text.value)"),
                                       ui.create_label(text="@binding(status_text.value)"), spacing=8)

        status_bar = ui.create_group(status_row)

        roi_combo_box = ui.create_combo_box(items_ref="roi_items", current_index="@binding(roi_item.value)")

        roi_row = ui.create_row(ui.create_label(text=_("ROI")), ui.create_stretch(), roi_combo_box, spacing=8)

        if len(ports) > 1:
            port_combo_box = ui.create_combo_box(items_ref="@binding(port_items.value)",
                                                 current_index="@binding(port_item.value)")
            port_row = ui.create_row(ui.create_label(text=_("Port")), ui.create_stretch(), port_combo_box, spacing=8)

        if camera_device.isMedipix:
            chips_combo_box = ui.create_combo_box(items_ref="@binding(chips_items.value)",
                                                  current_index="@binding(chips_item.value)")
            chips_row = ui.create_row(ui.create_label(text=_("Chips")), ui.create_stretch(), chips_combo_box, spacing=8)
        else:
            speed_combo_box = ui.create_combo_box(items_ref="@binding(speed_items.value)",
                                                  current_index="@binding(speed_item.value)")
            speed_row = ui.create_row(ui.create_label(text=_("Speed")), ui.create_stretch(), speed_combo_box, spacing=8)

        flip_value=ui.create_check_box(name='flip_value', text='Flip Spectra', checked='@binding(flip_model.value)')

        gain_combo_box = ui.create_combo_box(items_ref="@binding(gain_items.value)",
                                             current_index="@binding(gain_item.value)")

        preamp_row = ui.create_row(ui.create_label(text=_("Preamplifier")), ui.create_stretch(), gain_combo_box,
                                   spacing=8)

        if "Electron Multiplied" in ports or "EMCCD" in ports:
            multiplier_line_edit = ui.create_line_edit(
                text="@binding(multiplication_model.value, converter=multiplication_converter)")
            multiplier_row = ui.create_row(ui.create_label(text=_("Multiplier")), ui.create_stretch(),
                                           multiplier_line_edit, spacing=8)
            gain_column = ui.create_column(preamp_row, multiplier_row, spacing=8, margin=4)
        else:
            gain_column = ui.create_column(preamp_row, spacing=8, margin=4)

        gain_group = ui.create_group(gain_column, title=_("Gain"))

        correction_combo = ui.create_combo_box(items_ref="correction_items",
                                               current_index="@binding(correction_item.value)")
        correction_button = ui.create_push_button(name="correction_button", text=_("Measure"),
                                                  on_clicked="measure_clicked")
        correction_row = ui.create_row(ui.create_label(text=_("Method")), ui.create_stretch(), correction_button,
                                       ui.create_stretch(), correction_combo, spacing=8)

        threshold_line_edit = ui.create_line_edit(text="@binding(threshold_model.value, converter=threshold_converter)")
        threshold_row = ui.create_row(ui.create_label(text=_("Video threshold")), ui.create_stretch(),
                                      threshold_line_edit, spacing=8)
        threshold_column = ui.create_column(correction_row, threshold_row, spacing=8, margin=4)

        correction_group = ui.create_group(threshold_column, title=_("Corrections"))

        fan_checkbox = ui.create_check_box(name="fan_check_box", text="Fan Enabled",
                                           checked="@binding(fan_enabled_model.value)")

        # shutter_checkbox = ui.create_check_box(name="shutter_check_box", text="Shutter Enabled", checked="@binding(shutter_enabled_model.value)")

        if len(ports) > 1:
            if camera_device.isMedipix:
                setup_column = ui.create_column(roi_row, port_row, chips_row, flip_value, gain_group, correction_group,
                                                ui.create_stretch(), spacing=8, margin=4)
            else:
                setup_column = ui.create_column(roi_row, port_row, speed_row, flip_value, gain_group, correction_group,
                                                fan_checkbox, ui.create_stretch(), spacing=8, margin=4)
        else:
            setup_column = ui.create_column(roi_row, speed_row, gain_group, correction_group, fan_checkbox,
                                            ui.create_stretch(), spacing=8, margin=4)


    ############################### SPIM BEGIN #######################


        spim_column = ui.create_column(
            #ui.create_push_button(text='Prepare Spim', on_clicked='prepare_spim'),
            ui.create_label(text='Spim Test')
        )

        tabs = ui.create_tabs(ui.create_tab(_("Control"), control_column), ui.create_tab(_("Setup"), setup_column))

        tabs_column = ui.create_column(tabs, status_bar, margin=4)

        return tabs_column


def run():
    camera_panel_factory = CameraPanelFactory()

    Registry.register_component(camera_panel_factory, {"camera_panel"})
