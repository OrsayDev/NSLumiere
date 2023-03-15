import gettext
import enum
import typing
from ctypes import cdll, create_string_buffer, POINTER, byref
from ctypes import c_uint, c_int, c_char, c_char_p, c_void_p, c_short, c_long, c_bool, c_double, c_uint64, c_uint32, \
    Array, CFUNCTYPE, WINFUNCTYPE

import numpy

_ = gettext.gettext

__author__ = "Marcel Tencé"
__status__ = "alpha"
__version__ = "0.1"

#void(*newdriftCorrection)(void* scanobject, int scannb, double dx, double dy)
#NEWDRIFTCORRECTIONFUNC = WINFUNCTYPE(None, c_int, c_double, c_double)
#void(*newdriftmeasurement)(bool first, double drift_x, double drift_y, double delta_x, double delta_y, double quality, double time)
NEWDRIFTMEASUREMENTFUNC = WINFUNCTYPE(c_bool, c_double, c_double, c_double, c_double, c_double, c_double)

class OrsayDriftParameters:
    def __init__(self, mode: int, use_roi: bool, sum_refs: bool, search_width: int):
        self.__mode = mode
        self.__use_roi = use_roi
        self.__sum_refs = sum_refs
        self.__search_width = search_width

    @property
    def mode(self) -> int:
        return self.__mode

    @mode.setter
    def mode(self, mode: int):
        self.__mode = mode

    @property
    def use_roi(self) -> bool:
        return self.__use_roi

    @use_roi.setter
    def use_roi(self, on_off: bool):
        self.__use_roi = on_off

    @property
    def sum_refs(self) -> bool:
        return self.__sum_refs

    @sum_refs.setter
    def sum_refs(self, on_off: bool):
        self.__sum_refs = on_off

    @property
    def search_width(self) -> float:
        return self.__search_width

    @search_width.setter
    def search_width(self, width: float) -> None:
        self.__search_width = width


class drift_cancellation(object):

    def __build_function(self, str_call, args, result):
        if hasattr(self.__scan_dll, str_call):
            call = self.__scan_dll.__getattr__(str_call)
            call.argtypes = args
            call.restype = result
            return call
        else:
            raise Exception("*** " + str_call + " function not found in Orsay scan.dll ***")

    def __initialize_library(self, library) -> None:
        self.__scan_dll = library
        # void SCAN_EXPORT OrsayRegisterNewDriftMeasurement(void* o, void(*newdriftmeasurement)
        # (bool first, double drift_x, double drift_y, double delta_x, double delta_y, double quality, double time));
        self.__RegisterNewDriftMeasurement = \
            self.__build_function("OrsayRegisterNewDriftMeasurement",
                           [c_void_p, NEWDRIFTMEASUREMENTFUNC], None)
        # void SCAN_EXPORT OrsayRegisterNewDriftCorrection(void* o, void(*newdriftCorrection)(void* scanobject, int scannb, double dx, double dy));
        #
        # void SCAN_EXPORT OrsaySetDrift(void* o, int scannb, double dx, double dy);
        self.__SetDrift = self.__build_function("OrsaySetDrift",
                                         [c_void_p, c_int, c_double, c_double], None)
        # bool SCAN_EXPORT OrsayGetDrift(void* o, int scannb, double* dx, double* dy);
        self.__GetDrift = self.__build_function("OrsayGetDrift",
                                         [c_void_p, c_int, POINTER(c_double), POINTER(c_double)], c_bool)
        # void SCAN_EXPORT OrsayEnableDriftRoiCorrection(void* o, bool on_off);
        self.__EnableDriftRoiCorrection = self.__build_function("OrsayEnableDriftRoiCorrection",
                                                         [c_void_p, c_bool], None)
        # void SCAN_EXPORT OrsayDriftStart(void* o, int scannb, int sx, int sy);
        self.__DriftStart = self.__build_function("OrsayDriftStart", [c_void_p, c_bool, c_int, c_int], None)
        # void SCAN_EXPORT OrsayDriftStop(void* o);
        self.__DriftStop = self.__build_function("OrsayDriftStop", [c_void_p], None)
        # void SCAN_EXPORT OrsayDriftSetParameters(void* o, int mode, bool useROI, bool sumrefs, int searchwidth);
        self.__DriftSetParameters = self.__build_function("OrsayDriftSetParameters",
                                                   [c_void_p, c_int, c_bool, c_bool, c_int], None)
        # void SCAN_EXPORT OrsayDriftGetParameters(void* o, int* mode, bool* useROI, bool* sumrefs, int* searchwidth);
        self.__DriftGetParameters = self.__build_function("OrsayDriftGetParameters",
                                                   [c_void_p, POINTER(c_int), POINTER(c_bool), POINTER(c_bool),
                                                    POINTER(c_int)], None)
        # void SCAN_EXPORT OrsayDriftSetReferenceROI(void* o, int scannb, int xd, int xf, int yd, int yf);
        self.__DriftSetReferenceROI = self.__build_function("OrsayDriftSetReferenceROI",
                                                     [c_void_p, c_int, c_int, c_int, c_int], None)
        # bool SCAN_EXPORT OrsayDriftGetReferenceROI(void* o, int scannb, int* xd, int* xf, int* yd, int* yf);
        self.__DriftGetReferenceROI = self.__build_function("OrsayDriftGetReferenceROI",
                                                     [c_void_p, POINTER(c_int), POINTER(c_int), POINTER(c_int),
                                                      POINTER(c_int)], c_bool)
        # double SCAN_EXPORT OrsayDriftGetLinearModel(void* o, int average, double* driftx, double* drifty, double* xspeed, double* yspeed);
        self.__DriftGetLinearModel = self.__build_function("OrsayDriftGetLinearModel",
                                                    [c_void_p, c_int, POINTER(c_double), POINTER(c_double),
                                                     POINTER(c_double), POINTER(c_double)], c_double)
        # int SCAN_EXPORT OrsayGetDriftTableSize(void *o);
        self.__GetDriftTableSize = self.__build_function("OrsayGetDriftTableSize", [c_void_p], c_bool)
        # bool SCAN_EXPORT OrsayGetDriftAt(void *o, int n, double *x, double *y, double *t);
        self.__GetDriftAt = self.__build_function("OrsayGetDriftAt",
                                           [c_void_p, c_int, POINTER(c_double), POINTER(c_double), POINTER(c_double)],
                                           c_bool)

    def __init__(self, library):
        self.__initialize_library(library)

    """
    This function register a call back function to get the results of each drift measures
    It can then be displayed or whatever you like
    Call back function parameters
        first: True when it is the first measurement
        drift_x: cumulated drift on x axis
        drift_y: cumulated drift on y axis
        delta_x: drift on x since previous measurement
        delta_y: drift on y since previous measurement
        quality: correlation coefficient range[-1..1] a value > 0.5 is expected
        time: time of the measurement.
    """
    def register_new_drift_measurement(self, new_measure: NEWDRIFTMEASUREMENTFUNC) -> None:
        self.__RegisterNewDriftMeasurement(self.orsayscan, new_measure)
        #

    # bool GetDrift(int scannb, double& dx, double& dy);
    """
    This property get or set the current drift on both axis.
    setter should be used only if one calculate itself the drift.
    """
    @property
    def drift(self) -> tuple:
        dx = c_double()
        dy = c_double()
        self.__GetDrift(self.orsayscan, self.gene, dx, dy)
        return dx.value, dy.value

    @drift.setter
    def drift(self, delta) -> None:
        self.__SetDrift(self.orsayscan, self.gene, delta[0], delta[1])


    """
    This property tells if one want to measure drift only on the ROI.
    """
    @property
    def drift_roi_correction(self) -> bool:
        return self.__drift_roi_correction

    @drift_roi_correction.setter
    def drift_roi_correction(self, on_off: bool) -> None:
        self.__EnableDriftRoiCorrection(self.orsayscan, self.gene, on_off)
        self.__drift_roi_correction = on_off

    """
    This function enable the drift cancellation
    size_x: width of image, must same size as ROI if used otherwise size of the image
    size_y: height of image, must same size as ROI if used otherwise height of the image
    """
    def drift_start(self, size_x: int, size_y: int) -> None:
        self.__DriftStart(self.orsayscan, self.gene, size_x, size_y)

    """
    This function stop the drift cancellation
    """
    def drift_stop(self) -> None:
        self.__DriftStop()

    """
    Drift parameters are:
    mode: 0 drift cancellation is off
          1 drift is measured only (useful to check search algorithms)
          2 drift cancellation is active
    use_roi: when true, drift wil be measure only using roi, otherwise full image is taken.
    sum_refs: when true, all images are added to improve signal to noise in the reference image
    search_width: size of square where drift is look for. Useful to avoid collumn jump in periodic images.
    """
    @property
    def drift_parameters(self) -> OrsayDriftParameters:
        mode = c_int(0)
        use_roi = c_bool(False)
        sum_refs = c_bool(False)
        search_width = c_double(20)
        self.__DriftGetParameters(self.orsayscan, mode, use_roi, sum_refs, search_width)
        return OrsayDriftParameters(mode.value, use_roi.value, sum_refs.value, search_width.value)


    @drift_parameters.setter
    def drift_parameters(self, drift: OrsayDriftParameters) -> None:
        self.__DriftSetParameters(drift.mode, drift.use_roi, drift.sum_refs, drift.search_width)


    """
    This function define the roi used for drift measurement
    To work well, should be smaller that the ROI used in imaging when drift is measured or cancelled.
    start_x: first column of the roi
    end_x: last column of the roi, not included.
    """
    def drift_set_reference_roi(self, start_x:int, end_x:int, start_y:int, end_y:int) -> None:
        self.__DriftSetReferenceROI(self.orsayscan, start_x, end_x, start_y, end_y)


    def drift_get_reference_roi(self):
        start_x = c_int(0)
        end_x = c_int(10)
        start_y = c_int(0)
        end_y = c_int(10)
        self.__DriftGetReferenceROI(self.orsayscan, start_x, end_x, start_y, end_y)
        return start_x.value, end_x.value, start_y.value, end_y.value


    """
    return the drift parameter for the actual time
    It extrapolate from measurements, smoothing over the last values
    All units are pixels.
    average: number of measurements to use
    returns
    dx: drift on horizontal axis
    dy: drift on vertical axis
    spx: drift speed on horizontal axis
    spy: drift speed on vertical axis
    """
    def drift_get_linear_model(self, average:int):
        dx = c_double
        dy = c_double
        spx = c_double
        spy = c_double
        self.__DriftGetLinearModel(self.orsayscan, average, dx, dy, spx, spy)
        return (dx.value, dy.value), (spx.value, spy.value)