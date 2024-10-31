from nion.instrumentation import HardwareSource
from nion.utils import Registry
from nion.utils import Geometry
from nion.typeshed import API_1_0 as API
from nion.typeshed import UI_1_0 as UI
from nion.typeshed import Interactive_1_0 as Interactive

import time, numpy

api = api_broker.get_api(API.version, UI.version)  # type: API


def script_main(api_broker):
    interactive: Interactive.Interactive = api_broker.get_interactive(Interactive.version)
    api = api_broker.get_api(API.version, UI.version)  # type: API
    window = api.application.document_windows[0]

    #Getting the reference of necessary objects
    timepix3 = HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id(
        "orsay_camera_timepix3")
    main_controller = Registry.get_component("stem_controller")

    ### Getting the values from user ###
    xspim = interactive.get_integer("Number of pixels in X: ")
    yspim = interactive.get_integer("Number of pixels in Y: ")
    dwell_time = interactive.get_float("Acquisition time (s): ")

    #Getting the scan calibration
    fov_nm = main_controller.scan_controller.scan_device.current_frame_parameters.fov_nm
    dimensional_calibration_0 = api.create_calibration(-fov_nm/2, fov_nm / xspim, 'nm')
    dimensional_calibration_1 = api.create_calibration(-fov_nm / 2, fov_nm / yspim, 'nm')
    dimensional_calibration = [dimensional_calibration_0, dimensional_calibration_1]

    #Displaying the data item. It will be updated line by line in the control loop
    data = numpy.zeros((xspim, yspim))
    data_item = api.library.create_data_item_from_data(data)
    data_item.set_dimensional_calibrations(dimensional_calibration)
    data_item.title = "Timepix3Channel"
    _display_panel = window.display_data_item(data_item)

    should_break = False
    for ycur in range(yspim):
        if should_break: break
        for xcur in range(xspim):
            probe_position = Geometry.FloatPoint(ycur / yspim, xcur / xspim)
            main_controller.probe_position = probe_position
            timepix3.start_playing()
            should_break = interactive.cancelled
            if should_break: break
            time.sleep(dwell_time)
            frame = timepix3.grab_next_to_finish()[0]
            total_intensity = frame.data.sum()
            print(f"current at point {xcur + ycur * xspim} / {xspim * yspim}")
            data[ycur, xcur] = total_intensity
            timepix3.stop_playing()
            time.sleep(0.2)
        data_item.set_data(data)












# timepix3.start_playing()
# #scan.start_playing()
# #print(dir(scan.scan_device))
# print(main_controller.scan_controller.scan_device.scan_device_id)
# print(scan.scan_device.probe_pos)
# print(main_controller.probe_state)
# scan.scan_device.probe_pos = (0.5, 0.5)
# main_controller.probe_pos = (0.5, 0.5)
# print(scan.scan_device.probe_pos)
# print(main_controller.probe_position)
# time.sleep(0.5)
# timepix3.stop_playing()

