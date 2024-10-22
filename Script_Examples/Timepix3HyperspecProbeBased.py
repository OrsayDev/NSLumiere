from nion.instrumentation import HardwareSource
from nion.utils import Registry
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

    #Instatiating the data set
    data = numpy.zeros((xspim, yspim))

    #Getting the scan calibration
    fov_nm = main_controller.scan_controller.scan_device.current_frame_parameters.fov_nm
    dimensional_calibration_0 = api.create_calibration(-fov_nm/2, fov_nm / xspim, 'um')
    dimensional_calibration_1 = api.create_calibration(-fov_nm / 2, fov_nm / yspim, 'um')
    dimensional_calibration = [dimensional_calibration_0, dimensional_calibration_1]

    for xcur in range(xspim):
        for ycur in range(yspim):
            timepix3.start_playing()
            time.sleep(1)
            frame = timepix3.grab_next_to_finish()[0]
            total_intensity = frame.data.sum()
            data[xcur, ycur] = total_intensity
            main_controller.probe_pos = (xcur / xspim, ycur / yspim)
            timepix3.stop_playing()
            time.sleep(1.0)


    data_item = api.library.create_data_item_from_data(data)
    data_item.set_dimensional_calibrations(dimensional_calibration)
    data_item.title = "Timepix3Channel"
    _display_panel = window.display_data_item(data_item)







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

