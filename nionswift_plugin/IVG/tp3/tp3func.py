import json, time, requests, threading, logging, socket, numpy, struct, select
from numba import jit

from nion.swift.model import HardwareSource
from nion.utils import Registry

from ...aux_files import read_data

def SENDMYMESSAGEFUNC(sendmessagefunc):
    return sendmessagefunc

set_file = read_data.FileManager('camera_settings')
PIXELS_X = set_file.settings["Timepix3"]["PIXELS_X"]
PIXELS_Y = set_file.settings["Timepix3"]["PIXELS_Y"]
SPEC_SIZE_ISI = set_file.settings["Timepix3"]["SPEC_SIZE_ISI"]
SAVE_PATH = set_file.settings["Timepix3"]["SAVE_PATH"]
SAVE_PATH_TIFF = set_file.settings["Timepix3"]["SAVE_PATH_TIFF"]
PIXEL_MASK_PATH = set_file.settings["Timepix3"]["PIXEL_MASK_PATH"]
PIXEL_MASK_FILES = (set_file.settings["Timepix3"]["PIXEL_MASK_FILES"]).split()
PIXEL_THRESHOLD_FILES = (set_file.settings["Timepix3"]["PIXEL_THRESHOLD_FILES"]).split()
PIXEL_THRESHOLD_PATH = set_file.settings["Timepix3"]["PIXEL_THRESHOLD_PATH"]
BUFFER_SIZE = set_file.settings["Timepix3"]["BUFFER_SIZE"]
NUMBER_OF_MASKS = set_file.settings["Timepix3"]["NUMBER_OF_MASKS"]

#Modes that we receive a frame
FRAME = 0
FRAME_TR = 1
FASTCHRONO = 6
COINC_CHRONO = 7
FRAME_4DMASKED = 3
#Modes that we receive an event list
EVENT_HYPERSPEC = 2
EVENT_HYPERSPEC_COINC = 12
EVENT_4DRAW = 13
EVENT_LIST_SCAN = 14
#Modes in which the detector is in frame-based acquisition
FRAME_BASED = 10
HYPERSPEC_FRAME_BASED = 11
#Modes involving Isibox
ISIBOX_SAVEALL = 8

#TCP properties
PORT = 8088


@jit(nopython=True)
def update_spim_numba(array, event_list):
    for val in event_list:
        array[val] += 1

class Response:
    def __init__(self):
        self.text = '***TP3***: This is simul mode.'

class Timepix3Configurations:

    def __init__(self):
        self.settings = dict()

        self.bin = False
        self.bytedepth = 0
        self.cumul = False
        self.mode = 0
        self.xspim_size = 0
        self.yspim_size = 0
        self.xscan_size = 0
        self.yscan_size = 0
        self.pixel_time = 0
        self.time_delay = 0
        self.time_width = 0
        self.time_resolved = False
        self.save_locally = False
        self.pixel_mask = 0
        self.video_time = 0
        self.threshold = 0
        self.bias_voltage = 0
        self.destination_port = 0
        self.acquisition_us = 1000 #1 ms
        self.sup0 = 0.0
        self.sup1 = 0.0

    def __setattr__(self, key, value):
        try:
            self.settings[key] = value
            read_data.InstrumentDictSetter("Timepix3", key, value)
        except AttributeError:
            pass
        super(Timepix3Configurations, self).__setattr__(key, value)

    def create_configuration_bytes(self):
        return json.dumps(self.settings).encode()

    def get_array_size(self):
        shape = self.get_array_shape()
        array_size = 1
        try:
            for val in shape:
                array_size *= val
        except TypeError:
            array_size = shape
        return array_size

    def get_array_shape(self):
        if self.mode == FASTCHRONO:
            return self.xspim_size, PIXELS_X
        elif self.mode == COINC_CHRONO:
            return self.time_width * 2, PIXELS_X
        elif self.mode == FRAME or self.mode == FRAME_TR or self.mode == FRAME_BASED or self.mode == ISIBOX_SAVEALL:
            if self.bin:
                return PIXELS_X
            else:
                return PIXELS_Y, PIXELS_X
        elif self.mode == FRAME_4DMASKED:
            return self.yspim_size, self.xspim_size, NUMBER_OF_MASKS
        elif self.mode == EVENT_HYPERSPEC or self.mode == EVENT_HYPERSPEC_COINC or self.mode == EVENT_LIST_SCAN:
            return self.yspim_size, self.xspim_size, PIXELS_X
        elif self.mode == HYPERSPEC_FRAME_BASED: #Frame based measurement
            return self.yscan_size, self.xscan_size, PIXELS_X
        elif self.mode == EVENT_4DRAW:
            return self.yspim_size, self.xspim_size, PIXELS_Y, PIXELS_X
        else:
            raise TypeError(f"***TP3_CONFIG***: Attempted mode ({self.mode}) that is not configured in spimimage.")

    def get_data_receive_type(self):
        if self.bytedepth == 1:
            return numpy.dtype(numpy.uint8).newbyteorder('<')
        elif self.bytedepth == 2:
            return numpy.dtype(numpy.uint16).newbyteorder('<')
        elif self.bytedepth == 4:
            return numpy.dtype(numpy.uint32).newbyteorder('<')
        elif self.bytedepth == 8:
            return numpy.dtype(numpy.uint64).newbyteorder('<')


class Timepix3DataManager:
    def __init__(self):
        self.data = None

    def get_data(self, config: Timepix3Configurations):
        data_depth = config.get_data_receive_type()
        array_size = config.get_array_size()
        if config.mode == EVENT_HYPERSPEC or config.mode == EVENT_HYPERSPEC_COINC or config.mode == EVENT_LIST_SCAN:
            max_val = max(config.xspim_size, config.yspim_size)
            if max_val <= 64:
                self.data = numpy.zeros(array_size, dtype=numpy.uint32)
            elif max_val <= 1024:
                self.data = numpy.zeros(array_size, dtype=numpy.uint16)
            else:
                self.data = numpy.zeros(array_size, dtype=numpy.uint8)
        elif config.mode == EVENT_4DRAW:
            self.data = numpy.zeros(array_size, dtype=numpy.uint8)
        elif config.mode == FRAME or config.mode == FRAME_TR or config.mode == FRAME_BASED \
                or config.mode == FRAME_4DMASKED or config.mode == FASTCHRONO \
                or config.mode == COINC_CHRONO or config.mode == HYPERSPEC_FRAME_BASED or config.mode == ISIBOX_SAVEALL:
            self.data = numpy.zeros(array_size, dtype=data_depth)
        else:
            raise TypeError("***TP3_CONFIG***: Attempted mode ({self.mode}) that is not configured in get_data.")
        logging.info(f"***TP3_CONFIG***: Returning data for acquisition with shape {self.data.shape}.")
        return self.data

    def correct_data_or_not(self, gapsMode: int, config: Timepix3Configurations):
        if gapsMode == 1:
            if config.mode == FRAME or config.mode == FRAME_BASED:
                if config.bin:
                    for start_index in [255, 511, 767]:
                        self.data[start_index] = (2 * self.data[start_index - 1] + self.data[start_index + 2]) / 3.0
                        self.data[start_index + 1] = (2 * self.data[start_index + 2] + self.data[start_index - 1]) / 3.0
                else:
                    for start_index in [255, 511, 767]:
                        self.data[start_index::PIXELS_X] = (2 * self.data[start_index - 1::PIXELS_X] + self.data[start_index + 2::PIXELS_X]) / 3.0
                        self.data[start_index + 1::PIXELS_X] = (2 * self.data[start_index + 2::PIXELS_X] + self.data[start_index - 1::PIXELS_X]) / 3.0
            elif config.mode == COINC_CHRONO:
                for start_index in [255, 511, 767]:
                    self.data[start_index::PIXELS_X] = (2 * self.data[start_index - 1::PIXELS_X] + self.data[
                                                                                                     start_index + 2::PIXELS_X]) / 3.0
                    self.data[start_index + 1::PIXELS_X] = (2 * self.data[start_index + 2::PIXELS_X] + self.data[
                                                                                                         start_index - 1::PIXELS_X]) / 3.0
            else:
                logging.info("***TPX3***: No gap correction has been set for this mode.")

    def create_reshaped_array(self, config: Timepix3Configurations):
        shape = config.get_array_shape()
        return self.data.reshape(shape)

class TimePix3():

    def __init__(self, url, simul, message):

        fst_string = url.find('http://')
        assert fst_string==0, "***TP3***: Put ip_address in the form of 'http://IP:PORT'."
        sec_string = url.find(':', fst_string+7)

        self.success = False
        self.__serverURL = url
        self.__camIP = url[fst_string+7:sec_string]
        self.__data = None
        self.__frame = 0
        self.__spimData = None
        self.__detector_config = Timepix3Configurations()
        self.__data_manager = Timepix3DataManager()

        self.__frame_based = False
        self.__isPlaying = False
        self.__stem_controller = None
        self.__accumulation = 0.
        self.__expTime = 1.0
        self.__delay = 0.
        self.__width = 0.
        self.__subMode = 0.
        self.__gapsMode = 0
        self.__simul = simul
        self.__isReady = threading.Event()
        self.__binning = [1, 1]
        self.sendmessage = message

        self.__port = 0
        self.__dt = None
        self.__threshold = 0
        self.__pixelmask = 0

        # Loading bpc and dacs
        bpcFile = PIXEL_MASK_PATH + PIXEL_MASK_FILES[0]
        dacsFile = PIXEL_THRESHOLD_PATH + PIXEL_THRESHOLD_FILES[0]
        self.cam_init(bpcFile, dacsFile)
        self.acq_init()
        self.set_destination()

        if not simul:
            try:
                initial_status_code = self.status_code()
                if initial_status_code == 200:
                    logging.info('***TP3***: Timepix has initialized correctly.')
                else:
                    logging.info(f'***TP3***: Problem initializing Timepix3. Bad status code: {initial_status_code}.')

                logging.info(f'***TP3***: Current detector configuration is {self.get_config()}.')
                self.success = True
            except:
                logging.info('***TP3***: Problem initializing Timepix3. Cannot load files.')
        else:
            logging.info('***TP3***: Timepix3 in simulation mode.')

    def get_controller(self):
        # #TODO: In chromaTEM, this will overwrite superscan
        has_open_scan = (HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id(
            "open_scan_device") is not None)
        if has_open_scan:
            controller = HardwareSource.HardwareSourceManager().get_instrument_by_id("orsay_controller")
            assert controller.scan_controller.scan_device.scan_device_id == "open_scan_device"
            return controller
        if self.__stem_controller is None:
            return Registry.get_component("stem_controller")
        else:
            return self.__stem_controller

    def set_controller(self, controller):
        self.__stem_controller = controller

    def set_spec_output(self, exposure):
        scanInstrument = HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id(
            "orsay_scan_device")
        openscan = HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id(
            "open_scan_device")

        if openscan is not None:
            try:
                openscan.scan_device.scan_engine.mux_output_freq1 = 1.0 / exposure
            except AssertionError:
                logging.info("***TPX3***: Could not set the clock frequency. Setting it at 6 Hz (~166 ms) instead.")
                openscan.scan_device.scan_engine.mux_output_freq1 = 6.0
            openscan.scan_device.scan_engine.mux_output_type1 = 7
            openscan.scan_device.scan_engine.mux_output_freq_duty1 = 50
        else:
            logging.info("***TP3***: Cannot find openscan hardware. Tdc is not properly setted.")
        if scanInstrument is not None:
            scanInstrument.scan_device.orsayscan.SetTdcLine(1, 7, 0, period=exposure,)
            scanInstrument.scan_device.orsayscan.SetTdcLine(0, 2, 14)  # Laser trigger
        else:
            logging.info("***TP3***: Cannot find orsay scan hardware. Tdc is not properly setted.")


    def set_hyperspec_output(self):
        has_superscan = (HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id(
            "superscan") is not None)
        scanInstrument = HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id(
                "orsay_scan_device")
        openscan = HardwareSource.HardwareSourceManager().get_hardware_source_for_hardware_source_id(
            "open_scan_device")

        if openscan is not None:
            #TODO this overwrites superscan. You may want to copy superscan however
            # if has_superscan:
            #     openscan.scan_device.scan_engine.mux_output_type1 = 5
            #     openscan.scan_device.scan_engine.mux_output_pol1 = True
            # else:
            openscan.scan_device.scan_engine.mux_output_type1 = 3
            openscan.scan_device.scan_engine.mux_output_pol1 = True
            openscan.scan_device.scan_engine.flyback_us = 0
        else:
            logging.info("***TPX3***: Could not set the correct hyperspec output for openscan.")

        if scanInstrument is not None:
            if has_superscan:
                scanInstrument.scan_device.orsayscan.SetTdcLine(1, 2, 2)  # Copy Superscan input line.
                scanInstrument.scan_device.orsayscan.SetTdcLine(0, 2, 14)  # Laser trigger
            else:
                scanInstrument.scan_device.orsayscan.SetTdcLine(1, 2, 7)  # Copy Line Start.
                scanInstrument.scan_device.orsayscan.SetTdcLine(0, 2, 14)  # Laser trigger
        else:
            logging.info("***TPX3***: Could not set the correct hyperspec output for orsayscan.")

    def request_get(self, url):
        if not self.__simul:
            resp = requests.get(url=url)
            return resp
        else:
            resp = Response()
            return resp

    def request_put(self, url, data):
        if not self.__simul:
            resp = requests.put(url=url, data=data)
            return resp
        else:
            resp = Response()
            return resp

    def status_code(self):
        """
        Status code 200 is good. Other status code meaning can be seen in serval manual.
        """
        try:
            resp = self.request_get(url=self.__serverURL)
        except requests.exceptions.RequestException as e:  # Exceptions handling example
            return -1
        status_code = resp.status_code
        return status_code

    def dashboard(self):
        """
        Dashboard description can be seen in manual
        """
        resp = self.request_get(url=self.__serverURL + '/dashboard')
        data = resp.text
        dashboard = json.loads(data)
        return dashboard

    def cam_init(self, bpc_file, dacs_file):
        """
        This load both binary pixel config file and dacs.
        """
        resp = self.request_get(url=self.__serverURL + '/config/load?format=pixelconfig&file=' + bpc_file)
        data = resp.text
        logging.info(f'***TP3***: Response of loading binary pixel configuration file: ' + data)

        resp = self.request_get(url=self.__serverURL + '/config/load?format=dacs&file=' + dacs_file)
        data = resp.text
        logging.info(f'***TP3***: Response of loading dacs file: ' + data)

    def set_pixel_mask(self):
        which = self.__pixelmask
        self.__detector_config.pixel_mask = which
        if which < 8:
            bpcFile = PIXEL_MASK_PATH + PIXEL_MASK_FILES[which]
        else:
            logging.info(f'***TP3***: Pixel mask profile not found.')
            bpcFile = PIXEL_MASK_PATH + PIXEL_MASK_FILES[0]

        resp = self.request_get(url=self.__serverURL + '/config/load?format=pixelconfig&file=' + bpcFile)
        data = resp.text
        logging.info(f'***TP3***: Response of loading binary pixel configuration file (from set_pixel): ' + data)

    def set_threshold(self):
        which = self.__threshold
        self.__detector_config.threshold = which
        if which < 8:
            dacsFile = PIXEL_THRESHOLD_PATH + PIXEL_THRESHOLD_FILES[which]
        else:
            logging.info(f'***TP3***: Pixel mask profile not found.')
            dacsFile = PIXEL_THRESHOLD_PATH + PIXEL_THRESHOLD_FILES[0]

        resp = self.request_get(url=self.__serverURL + '/config/load?format=dacs&file=' + dacsFile)
        data = resp.text
        logging.info(f'***TP3***: Response of loading dacs file: ' + data)
        logging.info(f'***TP3***: Threshold is {which}.')

    def get_config(self):
        """
        Gets the entire detector configuration. Check serval manual to a full description.
        """
        if not self.__simul:
            resp = self.request_get(url=self.__serverURL + '/detector/config')
            data = resp.text
            detectorConfig = json.loads(data)
        else:
            detectorConfig = \
                {'Fan1PWM': 100, 'Fan2PWM': 100, 'BiasVoltage': 100, 'BiasEnabled': True, 'TriggerIn': 2,
                 'TriggerOut': 0,
                 'Polarity': 'Positive', 'TriggerMode': 'AUTOTRIGSTART_TIMERSTOP', 'ExposureTime': 0.05,
                 'TriggerPeriod': 0.05, 'nTriggers': 99999, 'PeriphClk80': False, 'TriggerDelay': 0.0,
                 'Tdc': ['P0', 'P0'], 'LogLevel': 1}
        return detectorConfig

    def set_exposure_time(self, exposure_time):
        self.__detector_config.acquisition_us = int(exposure_time * 1e6)
        detector_config = self.get_config()
        if self.__frame_based: #This is frame-based mode
            value = min(exposure_time, 0.250) #Maximum value is 250 ms
            detector_config["TriggerMode"] = "AUTOTRIGSTART_TIMERSTOP"
            detector_config["PeriphClk80"] = False
            detector_config["TriggerPeriod"] = value+self.getReadoutTime()  # 1s
            detector_config["ExposureTime"] = value  # 1s
        else:
            value = 0.1
            detector_config["PeriphClk80"] = False
            detector_config["TriggerMode"] = "CONTINUOUS"
            detector_config["TriggerPeriod"] = 0.1  # 1s
            detector_config["ExposureTime"] = 0.1  # 1s

        resp = self.request_put(url=self.__serverURL + '/detector/config', data=json.dumps(detector_config))
        data = resp.text
        logging.info(f'Response of updating Detector Configuration (exposure time to {value}): ' + data)

    def acq_init(self, ntrig=9999999):
        self.__detector_config.bias_voltage = 140
        """
        Initialization of detector. Standard value is 99999 triggers in continuous mode (a single trigger).
        """
        detector_config = self.get_config()
        detector_config["nTriggers"] = ntrig
        detector_config["TriggerMode"] = "CONTINUOUS"
        detector_config["TriggerOut"] = 2
        #detector_config["TriggerMode"] = "AUTOTRIGSTART_TIMERSTOP"
        detector_config["BiasEnabled"] = True
        detector_config["BiasVoltage"] = 140
        detector_config["Fan1PWM"] = 100 #100V
        detector_config["Fan2PWM"] = 100 #100V
        detector_config["TriggerPeriod"] = 1.0  # 1s
        detector_config["ExposureTime"] = 1.0  # 1s
        detector_config["Tdc"] = ['PN0', 'P0']

        resp = self.request_put(url=self.__serverURL + '/detector/config', data=json.dumps(detector_config))
        data = resp.text
        logging.info('Response of updating Detector Configuration: ' + data)

    def set_destination(self):
        port = self.__port
        self.__detector_config.destination_port = port
        """
        Sets the destination of the data. Data modes in ports are also defined here. Note that you always have
        data flown in port 8088 and 8089 but only one client at a time.
        """
        options = self.getPortNames()
        self.set_exposure_time(1.0) #Must set the correct trigger before updating destination
        if port==0 or port == 1:
            destination = {
                "Raw": [{
                    "Base": "tcp://connect@127.0.0.1:8098",
                }]
            }
        elif port == 2 or port == 3:
            destination = {
                "Raw": [{
                    #"Base": "file:/home/asi/load_files/data",
                    "Base": SAVE_PATH,
                    "FilePattern": "raw",
                }]
            }
        elif port == 4:
            destination = {
                "Raw": [{
                    "Base": "tcp://connect@127.0.0.1:8098",
                }],
                "Preview": {
                    "SamplingMode": "skipOnFrame",
                    "Period": 10000,
                    "ImageChannels": [{
                        "Base": SAVE_PATH_TIFF,
                        "FilePattern": "f%Hms_",
                        "Format": "tiff",
                        "Mode": "count_fb"
                    }]
                }
            }
        """
        elif port == 2:
            destination = {
                "Image": [{
                    "Base": "tcp://127.0.0.1:8088",
                    "Format": "jsonimage",
                    "Mode": "count",
                }]
            }
        elif port == 3:
            destination = {
                "Image": [{
                    "Base": "tcp://127.0.0.1:8088",
                    "Format": "jsonimage",
                    "Mode": "tot",
                }]
            }
        """
        resp = self.request_put(url=self.__serverURL + '/server/destination', data=json.dumps(destination))
        data = resp.text
        logging.info('***TP3***: Response of uploading the Destination Configuration to SERVAL : ' + data)
        logging.info(f'***TP3***: Selected port is {port} and corresponds to: ' + options[port])

    def getPortNames(self):
        return ['TCP Stream', 'TCP Stream + Save Locally', 'Save Locally', 'Save IsiBox Locally', 'Frame-based mode']

    def getCCDSize(self):
        return (256, 1024)

    def getSpeeds(self, port):
        return list(['Standard', 'Mask1', 'Mask2', 'Mask3', 'Mask4', 'Mask5', 'Mask6', 'Mask7'])

    def getGains(self, port):
        return list(['Very low', 'Low', 'Medium', 'High', 'Very high'])

    def getBinning(self):
        return self.__binning

    def setBinning(self, bx, by):
        self.__binning = [bx, by]

    def getImageSize(self):
        return (1025, 256)

    def registerLogger(self, fn):
        pass

    def addConnectionListener(self, fn):
        pass

    @property
    def simulation_mode(self) -> bool:
        return self.__simul

    def registerDataLocker(self, fn):
        pass

    def registerDataUnlocker(self, fn):
        pass

    def registerSpimDataLocker(self, fn):
        pass

    def registerSpimDataUnlocker(self, fn):
        pass

    def registerSpectrumDataLocker(self, fn):
        pass

    def registerSpectrumDataUnlocker(self, fn):
        pass

    def setCCDOverscan(self, sx, sy):
        pass

    def displayOverscan(self, displayed):
        pass

    def setMirror(self, mirror):
        pass

    def setAccumulationNumber(self, count):
        self.__accumulation = count

    def getAccumulateNumber(self):
        return int(self.__accumulation)

    def set_video_delay(self, value):
        value_in_640mhz = int(value) / 100 * 640
        self.__detector_config.video_time = int(value_in_640mhz)

    def setSpimMode(self, mode):
        pass

    def startFocus(self, exposure, displaymode, accumulate):
        """
        Start acquisition. Displaymode can be '1d' or '2d' and regulates the global attribute self.__softBinning.
        accumulate is 1 if Cumul and 0 if Focus. You use it to chose to which port the client will be listening on.
        Message=1 because it is the normal data_locker.
        """
        self.__isReady.clear()  # Clearing the Event so synchronization can occur properly later on
        self.set_exposure_time(exposure)
        self.set_spec_output(exposure)

        #Setting the configurations
        self.__detector_config.bin = True if displaymode == '1d' else False
        self.__detector_config.mode = 10 if self.__frame_based else int(self.__detector_config.time_resolved==True)
        self.__detector_config.cumul = bool(accumulate)
        if self.__port == 3:
            self.__detector_config.mode = 8
        self.__detector_config.bytedepth = 4
        self.__detector_config.xspim_size = self.__detector_config.xscan_size = self.getAccumulateNumber()
        self.__detector_config.yspim_size = self.__detector_config.yscan_size = self.getAccumulateNumber()
        self.__detector_config.pixel_time = self.get_scan_pixel_time()
        self.__detector_config.time_delay = int(self.__delay)
        self.__detector_config.time_width = int(self.__width)
        self.__detector_config.save_locally = (self.__port == 1)
        self.__detector_config.sup1 = self.get_detector_eels_dispersion()
        self.__detector_config.sup0 = self.get_detector_eels_offset()

        message = 1
        if self.getCCDStatus() == "DA_RECORDING":
            self.stopFocus()
        if self.getCCDStatus() == "DA_IDLE":
            resp = self.request_get(url=self.__serverURL + '/measurement/start')
            data = resp.text
            self.start_listening(message=message)
            return True
        else:
            logging.info('***TP3***: Check if experiment type matches mode selection.')

    def startChrono(self, exposure, displaymode, mode):
        """
        Start acquisition. Displaymode can be '1d' or '2d' and regulates the global attribute self.__softBinning.
        accumulate is 1 if Cumul and 0 if Focus. You use it to chose to which port the client will be listening on.
        Message=1 because it is the normal data_locker.
        """
        self.__isReady.clear()  # Clearing the Event so synchronization can occur properly later on
        self.set_spec_output(exposure),

        # Setting the configurations
        self.__detector_config.bin = True if displaymode == '1d' else False
        self.__detector_config.mode = 6 if mode == 0 else 7
        self.__detector_config.cumul = False if mode == 0 else True
        if self.__port == 3:
            self.__detector_config.mode = 8
        self.__detector_config.bytedepth = 4
        self.__detector_config.xspim_size = self.__detector_config.xscan_size = self.getAccumulateNumber()
        self.__detector_config.yspim_size = self.__detector_config.yscan_size = self.getAccumulateNumber()
        self.__detector_config.pixel_time = self.get_scan_pixel_time()
        self.__detector_config.time_delay = int(self.__delay)
        self.__detector_config.time_width = int(self.__width)
        self.__detector_config.save_locally = (self.__port == 1)
        self.__detector_config.sup1 = self.get_detector_eels_dispersion()
        self.__detector_config.sup0 = self.get_detector_eels_offset()

        message = 3
        if self.getCCDStatus() == "DA_RECORDING":
            self.stopFocus()
        if self.getCCDStatus() == "DA_IDLE":
            resp = self.request_get(url=self.__serverURL + '/measurement/start')
            data = resp.text
            self.start_listening(message=message)
            return True
        else:
            logging.info('***TP3***: Check if experiment type matches mode selection.')

    def startSpim(self, nbspectra, nbspectraperpixel, dwelltime, is2D):
        """
        Similar to startFocus. Just to be consistent with VGCameraYves. Message=02 because of spim.
        """
        self.__isReady.clear()  # Clearing the Event so synchronization can occur properly later on
        self.__frame_based = True
        self.set_exposure_time(dwelltime)

        # Setting the configurations
        self.__detector_config.bin = not is2D
        self.__detector_config.mode = 11
        self.__detector_config.cumul = False
        if self.__port == 3:
            self.__detector_config.mode = 8
        self.__detector_config.bytedepth = 4
        self.__detector_config.xspim_size = self.getAccumulateNumber()
        self.__detector_config.yspim_size = self.getAccumulateNumber()
        #You should call the function set_scan_size with the correct tuple. The total number of specimage is not enough
        #in the current implementation
        #self.__detector_config.scan_sizex, self.__detector_config.scan_sizey = self.get_scan_size()
        self.__detector_config.pixel_time = self.get_scan_pixel_time()
        self.__detector_config.time_delay = int(self.__delay)
        self.__detector_config.time_width = int(self.__width)
        self.__detector_config.save_locally = (self.__port == 1)
        self.__detector_config.sup1 = self.get_detector_eels_dispersion()
        self.__detector_config.sup0 = self.get_detector_eels_offset()

        message = 2
        if self.getCCDStatus() == "DA_RECORDING":
            self.stopFocus()
        if self.getCCDStatus() == "DA_IDLE":
            resp = self.request_get(url=self.__serverURL + '/measurement/start')
            data = resp.text
            self.start_listening(message=message)
            return True
        else:
            logging.info('***TP3***: Check if experiment type matches mode selection.')
        return


    def StartSpimFromScan(self):
        """
         This function must be called when you want to have a SPIM as a Scan Channel.
         """
        self.__isReady.clear() # Clearing the Event so synchronization can occur properly later on
        frame_parameters = self.get_scan_frame_parameters_and_abort()
        scanInstrument = self.get_controller().scan_controller
        self.set_hyperspec_output()

        # Setting the configurations
        if self.__port > 1: #This ensures we are streaming data and not saving locally
            self.setCurrentPort(0)

        self.__detector_config.bin = True
        self.__detector_config.mode = 2
        if self.__subMode == 0: #Standard
            self.__detector_config.mode = 2
        elif self.__subMode == 1: #Event-based in coincidence
            self.__detector_config.mode = 12
        elif self.__subMode == 2: #Event-based using raw 4D data
            self.__detector_config.mode = 13
        if scanInstrument.hardware_source_id == "open_scan_device":
            self.__detector_config.mode = 14
        self.__detector_config.cumul = False
        if self.__port == 3:
            self.__detector_config.mode = 8
        self.__detector_config.bytedepth = 8 if self.__subMode == 2 else 4
        self.__detector_config.xspim_size, self.__detector_config.yspim_size = self.get_scan_size(frame_parameters)
        self.__detector_config.xspim_size = self.__detector_config.xspim_size // self.__binning[0]
        self.__detector_config.yspim_size = self.__detector_config.yspim_size // self.__binning[0]
        self.__detector_config.xscan_size, self.__detector_config.yscan_size = self.get_scan_size(frame_parameters)
        self.__detector_config.pixel_time = self.get_scan_pixel_time()
        self.__detector_config.time_delay = int(self.__delay)
        self.__detector_config.time_width = int(self.__width)
        self.__detector_config.save_locally = (self.__port == 1)
        self.__detector_config.sup1 = self.get_detector_eels_dispersion()
        self.__detector_config.sup0 = self.get_detector_eels_offset()

        message = 2
        if self.getCCDStatus() == "DA_RECORDING":
            self.stopFocus()
            logging.info("***TPX3***: Please turn off TPX3 from camera panel. Trying to do it for you...")
            return False
        elif self.getCCDStatus() == "DA_IDLE":
            resp = self.request_get(url=self.__serverURL + '/measurement/start')
            data = resp.text
            self.start_listening_from_scan(message=message)
            return True

    def Start4DFromScan(self):
        """
         This function must be called when you want to have a SPIM as a Scan Channel.
         """
        self.__isReady.clear()  # Clearing the Event so synchronization can occur properly later on
        frame_parameters = self.get_scan_frame_parameters_and_abort()
        scanInstrument = self.get_controller().scan_controller
        self.set_hyperspec_output()

        # Setting the configurations
        if self.__port > 1: #This ensures we are streaming data and not saving locally
            self.setCurrentPort(0)

        self.__detector_config.bin = True
        self.__detector_config.mode = 3
        self.__detector_config.cumul = False
        if self.__port == 3:
            self.__detector_config.mode = 8
        self.__detector_config.bytedepth = 2
        self.__detector_config.xspim_size, self.__detector_config.yspim_size = self.get_scan_size(frame_parameters)
        self.__detector_config.xscan_size, self.__detector_config.yscan_size = self.get_scan_size(frame_parameters)
        self.__detector_config.pixel_time = self.get_scan_pixel_time()
        self.__detector_config.time_delay = int(self.__delay)
        self.__detector_config.time_width = int(self.__width)
        self.__detector_config.save_locally = (self.__port == 1)
        self.__detector_config.sup1 = self.get_detector_eels_dispersion()
        self.__detector_config.sup0 = self.get_detector_eels_offset()

        message = 4
        if self.getCCDStatus() == "DA_RECORDING":
            self.stopFocus()
            logging.info("***TPX3***: Please turn off TPX3 from camera panel. Trying to do it for you...")
        elif self.getCCDStatus() == "DA_IDLE":
            resp = self.request_get(url=self.__serverURL + '/measurement/start')
            data = resp.text
            self.start_4dlistening_from_scan()
            return True

    def stopTimepix3Measurement(self):
        status = self.getCCDStatus()
        resp = self.request_get(url=self.__serverURL + '/measurement/stop')
        return resp.text

    def stopFocus(self):
        """
        Stop acquisition. Finish listening put global isPlaying to False and wait client thread to finish properly using
        .join() method. Also replaces the old Queue with a new one with no itens on it (so next one won't use old data).
        """
        self.stopTimepix3Measurement()
        self.finish_listening()

    def stopSpim(self, immediate):
        """
        Identical to stopFocus. Just to be consistent with VGCameraYves.
        """
        self.stopFocus()

    def pauseSpim(self):
        pass

    def resumeSpim(self, mode):
        pass

    def isCameraThere(self):
        return True

    def getTemperature(self):
        pass

    def setTemperature(self, temperature):
        pass

    def setupBinning(self):
        pass

    def setExposureTime(self, exposure):
        """
        Set camera exposure time.
        """
        self.__expTime = exposure

    def setDelayTime(self, delay):
        self.__delay = delay

    def setWidthTime(self, width):
        self.__width = width

    def getNumofSpeeds(self, cameraport):
        pass

    def getCurrentSpeed(self, cameraport):
        return 999

    def getAllPortsParams(self):
        return None

    def setSpeed(self, cameraport, speed):
        self.__pixelmask = speed
        self.set_pixel_mask()

    def getNumofGains(self, cameraport):
        pass

    def getGain(self, cameraport):
        return 0

    def getGainName(self, cameraport, gain):
        pass

    def setGain(self, gain):
        self.__threshold = gain - 1
        self.set_threshold()

    def getReadoutTime(self):
        return 0.002

    def getNumofPorts(self):
        pass

    def getPortName(self, portnb):
        pass

    def getCurrentPort(self):
        if self.__port is not None:
            return self.__port
        else:
            return 999

    def setCurrentPort(self, cameraport):
        self.__port = cameraport
        self.__frame_based = True if (self.__port == 4) else False
        self.set_destination()

    def getMultiplication(self):
        return [1]

    def setMultiplication(self, multiplication):
        pass

    def getCCDStatus(self) -> dict():
        '''
        Returns
        -------
        str

        Notes
        -----
        DA_IDLE is idle. DA_PREPARING is busy to setup recording. DA_RECORDING is busy recording
        and output data to destinations. DA_STOPPING is busy to stop the recording process
        '''
        if not self.__simul:
            dashboard = json.loads(self.request_get(url=self.__serverURL + '/dashboard').text)
            if dashboard["Measurement"] is None:
                return "DA_IDLE"
            else:
                return dashboard["Measurement"]["Status"]
        else:
            value = "DA_RECORDING" if self.__isPlaying else "DA_IDLE"
            return value

    def getReadoutSpeed(self):
        pass

    def getPixelTime(self, cameraport, speed):
        pass

    def adjustOverscan(self, sizex, sizey):
        pass

    def setTurboMode(self, active, sizex, sizey):
        pass

    def getTurboMode(self):
        return [0]

    def setExposureMode(self, mode, edge):
        pass

    def getExposureMode(self):
        pass

    def setPulseMode(self, mode):
        pass

    def setVerticalShift(self, shift, clear):
        pass

    def setFan(self, On_Off: bool):
        pass

    def getFan(self):
        return False

    def setArea(self, area: tuple):
        pass

    def getArea(self):
        return (0, 0, 256, 1024)

    def setVideoThreshold(self, threshold):
        pass

    def getVideoThreshold(self):
        pass

    def setCCDOverscan(self, sx, sy):
        pass

    def setTp3Mode(self, value):
        self.__subMode = value

    def setTp3BiasVoltage(self, value):
        """
        Setting the bias voltage of the detector.
        """
        if float(value) < 200.0:
            self.__detector_config.bias_voltage = int(value)
            detector_config = self.get_config()
            detector_config["BiasVoltage"] = float(value)
            resp = self.request_put(url=self.__serverURL + '/detector/config', data=json.dumps(detector_config))
            data = resp.text
            logging.info('***TPX3***: Response of updating Detector Configuration (bias voltage only): ' + data)
        else:
            logging.info("***TPX3***: Bias voltage must be below 200.0 V.")

    def setTp3TR(self, value: bool):
        """
        Set if the measurement is TR or not
        """
        self.__detector_config.time_resolved = value

    def getTp3Modes(self):
        return ['Standard', 'Coincidence', 'Raw 4D Image']

    @property
    def gaps_mode(self)-> int:
        return self.__gapsMode

    @gaps_mode.setter
    def gaps_mode(self, value: int):
        self.__gapsMode = value

    def start_listening(self, message=1):
        """
        Starts the client Thread and sets isPlaying to True.
        """
        self.__isPlaying = True
        self.__clientThread = threading.Thread(target=self.acquire_streamed_frame, args=(message,))
        self.__clientThread.start()

    def start_listening_from_scan(self, message=1):
        """
        Starts the client Thread and sets isPlaying to True.
        """
        self.__isPlaying = True
        self.__clientThread = threading.Thread(target=self.acquire_streamed_frame_from_scan)
        self.__clientThread.start()

    def start_4dlistening_from_scan(self):
        """
        Starts the client Thread and sets isPlaying to True.
        """
        self.__isPlaying = True
        self.__clientThread = threading.Thread(target=self.acquire_4dstreamed_frame_from_scan_frame)
        self.__clientThread.start()


    def finish_listening(self):
        """
        .join() the client Thread, puts isPlaying to false and replaces old queue to a new one with no itens on it.
        """
        if self.__isPlaying:
            self.__isPlaying = False
            self.__clientThread.join()
            logging.info(f'***TP3***: Stopping acquisition.')

    def save_locally_routine(self):
        logging.info(
            '***TP3***: Save locally is activated. No socket will be open. Line start and line 05 is sent to TDC.')
        self.set_hyperspec_output()


    def set_scan_size(self, value: (int, int)):
        self.__detector_config.yscan_size, self.__detector_config.xscan_size = value

    def get_scan_frame_parameters_and_abort(self):
        scanInstrument = self.get_controller().scan_controller
        scanInstrument.start_playing()
        time.sleep(0.5)
        on_frame_parameters = scanInstrument.scan_device.current_frame_parameters
        scanInstrument.stop_playing()
        while scanInstrument.is_playing:
            logging.info("***TP3***: Waiting for the scan to stop.")
            time.sleep(0.1)
        return on_frame_parameters

    def get_scan_size(self, frame_parameters):
        if frame_parameters.subscan_pixel_size:
            x_size = int(frame_parameters.subscan_pixel_size[1])
            y_size = int(frame_parameters.subscan_pixel_size[0])
        else:
            x_size = int(frame_parameters.size[1])
            y_size = int(frame_parameters.size[0])
        return x_size, y_size

    def get_scan_pixel_time(self) -> int:
        try:
            return int(
                self.get_controller().scan_controller.scan_device.current_frame_parameters.pixel_time_us * 1000 / 1.5625)
        except AttributeError:
            logging.info("***TPX3***: Could not retrieve the scan pixel time. Returning 0 instead.")
            return 0

    def get_detector_eels_dispersion(self) -> float:
        return float(self.get_controller().TryGetVal("EELS_TV_eVperpixel")[1])

    def get_detector_eels_offset(self) -> float:
        return float(self.get_controller().TryGetVal("KURO_EELS_eVOffset")[1])

    def _prepare_for_acquisition(self):
        if self.__port==2:
            self.save_locally_routine()
            return
        elif self.__port == 3:
            self.save_locally_routine()

        config_bytes = self.__detector_config.create_configuration_bytes()

        # Connecting the socket.
        inputs = list()
        outputs = list()
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        """
        127.0.0.1 -> LocalHost;
        129.175.108.58 -> Patrick;
        129.175.81.162 -> My personal Dell PC;
        192.0.0.11 -> My old personal (outside lps.intra);
        192.168.199.11 -> Cheetah (to VG Lum. Outisde lps.intra);
        129.175.108.52 -> CheeTah
        """
        ip = socket.gethostbyname('127.0.0.1') if self.__simul else socket.gethostbyname(self.__camIP)
        address = (ip, PORT)
        try:
            client.connect(address)
            logging.info(f'***TP3***: Both clients connected over {ip}:{PORT}. Acquiring streamed frame.')
            inputs.append(client)
        except ConnectionRefusedError:
            self.stopTimepix3Measurement()
            return False

        self.__dt = self.__detector_config.get_data_receive_type()
        self.__data = self.__data_manager.get_data(self.__detector_config)
        client.send(config_bytes)

        logging.info(f"Creating data type as {self.__dt} and array shape with {self.__data.shape} for the acquisition.")

        self.__isReady.set()  # This waits until spimData is created so scan can have access to it.

        return inputs, outputs

    def acquire_streamed_frame(self, message):
        """
        Main client function. Main loop is explained below.

        Client is a socket connected to camera in host computer 129.175.108.52. Port depends on which kind of data you
        are listening on. After connection, timeout is set to 5 ms, which is camera current dead time. cam_properties
        is a dict containing all info camera sends through tcp (the header); frame_data is the frame; buffer_size is how
        many bytes we collect within each loop interaction; frame_number is the frame counter and frame_time is when the
        whole frame began.

        check string value is a convenient function to detect the values using the header standard format for jsonimage.
        """

        cam_properties = dict()
        inputs, outputs = self._prepare_for_acquisition()

        def check_string_value(header, prop):
            """
            Check the value in the header dictionary. Some values are not number so a valueError
            exception handles this.
            """

            start_index = header.index(prop)
            end_index = start_index + len(prop)
            begin_value = header.index(':', end_index, len(header)) + 1
            if prop == 'height':
                end_value = header.index('}', end_index, len(header))
            else:
                end_value = header.index(',', end_index, len(header))
            try:
                if prop == 'timeAtFrame':
                    value = float(header[begin_value:end_value])
                else:
                    value = int(header[begin_value:end_value])
            except ValueError:
                value = str(header[begin_value:end_value])
            return value

        def check_data_and_send_message(cam_prop, frame):
            try:
                assert int(cam_properties['width']) * int(
                    cam_properties['height']) * int(
                    cam_properties['bitDepth'] / 8) == int(cam_properties['dataSize'])
                assert cam_properties['dataSize'] == len(frame)
                self.sendmessage(message)
                return True
            except AssertionError:
                logging.info(
                    f'***TP3***: Problem in size/len assertion. Properties are {cam_properties} and data is {len(frame)}')
                return False

        while True:
            try:
                read, _, _ = select.select(inputs, outputs, inputs)
                for s in read:
                    if s == inputs[0]:
                        packet_data = s.recv(128)
                        if packet_data == b'':
                            logging.info("***TP3***: No data received. Closing connection.")
                            self.stopTimepix3Measurement()
                            return
                        while (packet_data.find(b'{"time') == -1) or (packet_data.find(b'}\n') == -1):
                            temp = s.recv(BUFFER_SIZE)
                            if temp == b'':
                                logging.info("***TP3***: No data received. Closing connection.")
                                self.stopTimepix3Measurement()
                                return
                            else:
                                packet_data += temp

                        begin_header = packet_data.index(b'{"time')
                        end_header = packet_data.index(b'}\n', begin_header)
                        header = packet_data[begin_header:end_header + 1].decode('latin-1')

                        for properties in ["timeAtFrame", "frameNumber", "measurementID", "dataSize", "bitDepth",
                                           "width",
                                           "height"]:
                            cam_properties[properties] = (check_string_value(header, properties))

                        data_size = int(cam_properties['dataSize'])

                        #Confirming that the array size is coherent
                        shape = self.__detector_config.get_array_shape()
                        size = self.__detector_config.get_array_size()
                        if size == 2:
                            assert int(cam_properties['width']) == shape[1], \
                                 "***TPX3***: The width of the detector is not what is expected. Check the JSON file."
                            assert int(cam_properties['height']) == shape[0], \
                                "***TPX3***: The height of the detector is not what is expected. Check the JSON file."
                        elif size == 1:
                            assert int(cam_properties['width']) == shape[0], \
                                "***TPX3***: The width of the detector is not what is expected. Check the JSON file."
                            assert int(cam_properties['height']) == 1, \
                                "***TPX3***: The height of the detector is not what is expected. Check the JSON file."

                        assert (begin_header == 0)
                        how_many_more_bytes = data_size + len(header) - len(packet_data) + 1
                        while how_many_more_bytes != 0:
                            bytes_to_receive = min(BUFFER_SIZE, how_many_more_bytes)
                            packet_data += s.recv(bytes_to_receive)
                            how_many_more_bytes = data_size + len(header) - len(packet_data) + 1

                        frame_data = packet_data[end_header + 2:end_header + 2 + data_size]

                        event_list = numpy.frombuffer(frame_data, dtype=self.__dt)
                        if message == 1 or message == 3:
                            self.__data[:] = event_list[:]
                            self.__frame = cam_properties['frameNumber']
                            self.__data_manager.correct_data_or_not(self.__gapsMode, self.__detector_config)
                            check_data_and_send_message(cam_properties, frame_data)
                        if message == 2:
                            start_channel = int(cam_properties['frameNumber']) * PIXELS_X #Spatial pixel * number of energy channels
                            number_of_channels = int((data_size * 8 / int(cam_properties['bitDepth'])))
                            extra_pixels = int(number_of_channels / PIXELS_X)
                            self.__data[start_channel:start_channel + number_of_channels] = event_list[:]
                            #print(f'***TP3***: Acquiring hyperspecimage. Header is {header}. Start and number of channels is {start_channel} and {number_of_channels}. Number of pixels per call is {extra_pixels}.')
                            self.__frame = min(cam_properties['frameNumber'] + extra_pixels, self.__detector_config.xscan_size * self.__detector_config.yscan_size)
                            self.sendmessage(2)
                            if start_channel + number_of_channels >= self.__detector_config.xscan_size * self.__detector_config.yscan_size * PIXELS_X:
                                self.stopTimepix3Measurement()
                                logging.info("***TP3***: Spim is over. Closing connection.")
                                return

            except ConnectionResetError:
                self.stopTimepix3Measurement()
                logging.info("***TP3***: Socket reseted. Closing connection.")
                return
            if not self.__isPlaying:
                self.stopTimepix3Measurement()
                logging.info("***TP3***: Not playing. Closing connection.")
                return
        return

    def acquire_streamed_frame_from_scan(self):
        """
        Main client function. Main loop is explained below.

        Client is a socket connected to camera in host computer 129.175.108.52. Port depends on which kind of data you
        are listening on. After connection, timeout is set to 5 ms, which is camera current dead time. cam_properties
        is a dict containing all info camera sends through tcp (the header); frame_data is the frame; buffer_size is how
        many bytes we collect within each loop interaction; frame_number is the frame counter and frame_time is when the
        whole frame began.

        check string value is a convenient function to detect the values using the header standard format for jsonimage.
        """

        inputs, outputs = self._prepare_for_acquisition()
        scanInstrument = self.get_controller().scan_controller
        total_frames = self.getAccumulateNumber()
        start = time.time()
        logging.info(f'***TPX3***: Number of frames to be acquired is {total_frames}.')

        try:
            scanInstrument.set_sequence_buffer_size(total_frames)
            scanInstrument.start_sequence_mode(scanInstrument.get_current_frame_parameters(), total_frames)
        except AssertionError:
            logging.info(f"***TPX3***: Could not set the buffer size on the scan device.")
            self.stopTimepix3Measurement()
            return

        # If its a list scan, you should inform the value
        # TODO: random scan works here, but does not look the optimal emplacement.
        if scanInstrument.hardware_source_id == "open_scan_device":
            array_to_send = scanInstrument.scan_device.scan_engine.get_ordered_array().astype('uint32')
            inputs[0].sendall(array_to_send)

        while True:
            try:
                read, _, _ = select.select(inputs, outputs, inputs)
                for s in read:
                    packet_data = s.recv(BUFFER_SIZE)
                    if len(packet_data) == 0:
                        logging.info('***TP3***: No more packets received. Finishing SPIM.')
                        self.stopTimepix3Measurement()
                        return

                    #Checking if its a multiple of 8 bytes (64 bit)
                    q = len(packet_data) % 8
                    if q:
                        packet_data += s.recv(8 - q)

                    try:
                        event_list = numpy.frombuffer(packet_data, dtype=self.__dt)
                        update_spim_numba(self.__data, event_list)
                    except ValueError:
                        logging.info(f'***TP3***: Value error.')
                    except IndexError:
                        logging.info(f'***TP3***: Indexing error.')

            except ConnectionResetError:
                logging.info("***TP3***: Socket reseted. Closing connection.")
                self.stopTimepix3Measurement()
                return

            if not self.__isPlaying or not scanInstrument.is_playing:
                self.stopTimepix3Measurement()
                logging.info('***TP3***: Scanning is halted. Finishing SPIM.')
                return
            read_frames = scanInstrument.get_sequence_buffer_count()
            if time.time() - start > 5.0:
                start = time.time()
                logging.info(f'***TP3***: Number of scans performed is {read_frames} out of {total_frames}.')
            if read_frames >= total_frames:
                self.stopTimepix3Measurement()
                scanInstrument.stop_playing()
                logging.info('***TP3***: Buffer complete. Halting scan.')
                return
        return

    def acquire_4dstreamed_frame_from_scan_frame(self):
        """
        Main client function. Main loop is explained below.

        Client is a socket connected to camera in host computer 129.175.108.52. Port depends on which kind of data you
        are listening on. After connection, timeout is set to 5 ms, which is camera current dead time. cam_properties
        is a dict containing all info camera sends through tcp (the header); frame_data is the frame; buffer_size is how
        many bytes we collect within each loop interaction; frame_number is the frame counter and frame_time is when the
        whole frame began.

        check string value is a convenient function to detect the values using the header standard format for jsonimage.
        """

        inputs, outputs = self._prepare_for_acquisition()
        scanInstrument = self.get_controller().scan_controller
        total_frames = self.getAccumulateNumber()
        start = time.time()
        logging.info(f'***TPX3***: Number of frames to be acquired is {total_frames}.')

        # If its a list scan, you should inform the value
        # TODO: random scan does not work here because we change the list after starting the sequence below.
        if scanInstrument.hardware_source_id == "open_scan_device":
            array_to_send = scanInstrument.scan_device.scan_engine.get_ordered_array().astype('uint32')
            inputs[0].sendall(array_to_send)

        try:
            scanInstrument.set_sequence_buffer_size(total_frames)
            scanInstrument.start_sequence_mode(scanInstrument.get_current_frame_parameters(), total_frames)
        except AssertionError:
            logging.info(f"***TPX3***: Could not set the buffer size on the scan device.")
            self.stopTimepix3Measurement()
            return

        while True:
            try:
                read, _, _ = select.select(inputs, outputs, inputs)
                for s in read:
                    packet_data = s.recv(BUFFER_SIZE)

                    if len(packet_data) == 0:
                        logging.info('***TP3***: No more packets received. Finishing SPIM.')
                        return

                    how_many_more_bytes = self.__detector_config.xscan_size * \
                                          self.__detector_config.yscan_size * \
                                          NUMBER_OF_MASKS * 2 - len(packet_data)
                    while how_many_more_bytes != 0:
                        bytes_to_receive = min(BUFFER_SIZE, how_many_more_bytes)
                        packet_data += s.recv(bytes_to_receive)
                        how_many_more_bytes = self.__detector_config.xscan_size * \
                                              self.__detector_config.yscan_size * \
                                              NUMBER_OF_MASKS * 2 - len(packet_data)
                    try:
                        event_list = numpy.frombuffer(packet_data, dtype=dt)
                        self.__data[:] = event_list[:]
                    except ValueError:
                        logging.info(f'***TP3***: Value error.')
                    except IndexError:
                        logging.info(f'***TP3***: Indexing error.')

            except ConnectionResetError:
                logging.info("***TP3***: Socket reseted. Closing connection.")
                return

            if not self.__isPlaying or not scanInstrument.is_playing:
                self.stopTimepix3Measurement()
                logging.info('***TP3***: Scanning is halted. Finishing SPIM.')
                return
        return

    def update_spim(self, event_list):
        unique, counts = numpy.unique(event_list, return_counts=True)
        counts = counts.astype(numpy.uint32)
        self.__data[unique] += counts

    def get_current(self, frame_int, frame_number):
        if self.__detector_config.cumul and frame_number:
            eps = (numpy.sum(frame_int) / self.__expTime) / frame_number
        else:
            eps = numpy.sum(frame_int) / self.__expTime
        cur_pa = eps / (6.242 * 1e18) * 1e12
        return cur_pa

    def create_specimage(self):
        return self.__data_manager.create_reshaped_array(self.__detector_config)

    def create_spimimage(self):
        return self.__data_manager.create_reshaped_array(self.__detector_config)

    def get_frame(self):
        return self.__frame

    def create_spimimage_frame(self):
        return self.__data.reshape((self.__detector_config.yscan_size, self.__detector_config.xscan_size, PIXELS_X))

    def create_4dimage(self):
        return self.__data_manager.create_reshaped_array(self.__detector_config)
