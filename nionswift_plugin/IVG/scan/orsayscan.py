"""
Class controlling orsay scan hardware.
"""
import sys, os
from ctypes import cdll, create_string_buffer, POINTER, byref
from ctypes import c_uint, c_int, c_char, c_char_p, c_void_p, c_short, c_long, c_bool, c_double, c_uint64, c_uint32, Array, CFUNCTYPE, WINFUNCTYPE
import typing
from . import scan_pattern, drift_cancellation

__author__  = "Marcel Tence & Yves Auad"
__status__  = "alpha"
__version__ = "0.1"

def _isPython3():
    return sys.version_info[0] >= 3

def _buildFunction(call, args, result):
    call.argtypes = args
    call.restype = result
    return call

def _createCharBuffer23(size):
    if (_isPython3()):
        return create_string_buffer(b'\000' * size)
    return create_string_buffer('\000' * size)

def _convertToString23(binaryString):
    if (_isPython3()):
        return binaryString.decode("utf-8")
    return binaryString

def _toString23(string):
    if (_isPython3()):
        return string.encode("utf-8")
    return string

#is64bit = sys.maxsize > 2**32
if (sys.maxsize > 2**32):
    libname = os.path.join(os.path.dirname(__file__), "../../aux_files/DLLs/Scan.dll")
    _library = cdll.LoadLibrary(libname)
    #print(f"OrsayScan library: {_library}")
else:
    raise Exception("It must a python 64 bit version")

# void *(*LockScanDataPointer)(int gene, int *datatype, int *sx, int *sy, int *sz);
LOCKERFUNC = WINFUNCTYPE(c_void_p, c_int, POINTER(c_int), POINTER(c_int), POINTER(c_int), POINTER(c_int))
# void(*UnLockScanDataPointer)(int gene, bool newdata);
UNLOCKERFUNC = WINFUNCTYPE(None, c_int, c_bool)
UNLOCKERFUNCA = WINFUNCTYPE(None, c_int, c_int, c_int, POINTER(c_int))

EELS_SCAN_CLOCK = 2
CL_SCAN_CLOCK = 4

class scan_input(object):
    """
    input definition and properties from scan.xml as read by dll C++
    name => label for the input - read only
    index => physical input number of the scan box - read only
    unipolar => True when input configured as unipolar, otherwise bipolar. (only bipolar fully tested now)
    offset => Zero adjust in Volts, should be near 0 for bipolar, may be as large as -2.5V for unipolar
    """

    def __init__(self, name: str, index: int, unipolar: bool, offset: float, enabled: bool):
        self.__name = name
        self.__unipolar = unipolar
        self.__index = index
        self.__offset = offset
        self.__enabled = enabled

    @property
    def name(self) -> str:
        return self.__name

    @property
    def index(self) -> int:
        return self.__index

    @property
    def unipolar(self) -> bool:
        return self.__unipolar

    @property
    def offset(self) -> float:
        return self.__offset

    @offset.setter
    def offset(self, value: float) -> None:
        self.__offset = value

    @property
    def enabled(self) -> bool:
        return self.__enabled

    def as_dict(self) -> typing.Dict[str, typing.Any]:
        d: typing.Dict[str, typing.Any] = {
            "name": self.__name,
            "index": self.__index,
            "unipolar": self.__unipolar,
            "offset": self.__offset,
            "enabled": self.__enabled
        }
        return d

class orsayScan(object):
    """Class controlling orsay scan hardware
       Requires Scan.dll library to run.
    """

    def __build_function(self, str_call, args, result):
        if hasattr(self.__scan_dll, str_call):
            call = self.__scan_dll.__getattr__(str_call)
            call.argtypes = args
            call.restype = result
            return call
        else:
            raise Exception("*** " + str_call + " function not found in " + self.__libname + " ***")

    def __load_library(self):
        if sys.platform.startswith("win") and (sys.maxsize > 2 ** 32):
            # check if library configuration file is present
            # if not copy default configuration
            programdata = os.getenv("ProgramData")
            microscope_path = os.path.join(os.getenv("ProgramData"), "Microscope")
            ### From Yves ###
            """
            if not os.path.exists(microscope_path):
                os.makedirs(microscope_path)
            scan_path = os.path.join(microscope_path, "Scan")
            if not os.path.exists:
                os.makedirs(scan_path)
            if not os.path.exists(os.path.join(scan_path, "scan.xml")):
                copy2("scan.xml", scan_path)
            if not os.path.exists(os.path.join(scan_path, "Simulation.xml")):
                copy2("Simulation.xml", scan_path)
            self.__libname = os.path.join(os.path.dirname(__file__), "Scan.dll")
            """
            ### From Yves ###
            self.__libname = os.path.join(os.path.dirname(__file__), "../../aux_files/DLLs/Scan.dll")
            print(self.__libname)
            # copy required dlls to application folder
            self.__python_folder = os.path.dirname(sys.executable)
            # if self.__python_folder is not None:
            #     copyfile_if_newer("udk3-1.5.1-x86_64.dll", self.__python_folder)
            #     copyfile_if_newer("udk3mod-1.5.1-winusb-x86_64.dll", self.__python_folder)
            self.__scan_dll = cdll.LoadLibrary(self.__libname)
        else:
            raise Exception("This module is only python 64 bit on windows compatible")

    def __initialize(self) -> None:
        """
        Make all direct dll function as protected.
        """
        self.__load_library()
        # void SCAN_EXPORT *OrsayScanInit();
        self.__ScanInit = self.__build_function("OrsayScanInit", [c_bool, c_bool], c_void_p)

        # void SCAN_EXPORT *OrsayScanInitEx(bool nion, const char* libfolder);
        self.__InitEx = self.__build_function("OrsayScanInitEx", [c_bool, c_char_p, c_bool], c_void_p)

        # void SCAN_EXPORT OrsayScanClose(void* o)
        self.__Close = self.__build_function("OrsayScanClose", [c_void_p], None)

        # void SCAN_EXPORT OrsayScangetVersion(void* o, short *product, short *revision, short *serialnumber, short *major, short *minor);
        self.__getVersion = self.__build_function("OrsayScangetVersion",
                                                  [c_void_p, POINTER(c_short), POINTER(c_short), POINTER(c_short),
                                                   POINTER(c_short), POINTER(c_short)], None)

        # int SCAN_EXPORT OrsayScanGetInputsCount(void* o);
        self.__getInputsCount = self.__build_function("OrsayScanGetInputsCount", [c_void_p], c_int)

        # int SCAN_EXPORT OrsayScanGetInputProperties(void* o, int nb, bool &unipolar, double &offset, char *buffer);
        self.__GetInputProperties = self.__build_function("OrsayScanGetInputProperties",
                                                          [c_void_p, c_int, POINTER(c_bool), POINTER(c_double),
                                                           c_char_p],
                                                          c_int)

        #	bool SCAN_EXPORT OrsayScanSetInputProperties(void* o, int nb, bool unipolar, double offset);
        self.__SetInputProperties = self.__build_function("OrsayScanSetInputProperties",
                                                          [c_void_p, c_int, c_bool, c_double], c_bool)

        # bool SCAN_EXPORT OrsayScansetImageSize(void *o, int gene, int x, int y);
        self.__setImageSize = self.__build_function("OrsayScansetImageSize",
                                                    [c_void_p, c_int, c_int, c_int],
                                                    c_bool)

        #	bool SCAN_EXPORT OrsayScangetImageSize(void *o, int gene, int *x, int *y);
        self.__getImageSize = self.__build_function("OrsayScangetImageSize",
                                                    [c_void_p, c_int, POINTER(c_int), POINTER(c_int)], c_bool)

        # bool SCAN_EXPORT OrsayScansetImageArea(void* o, int gene, int sx, int sy, int xd, int xf, int yd, int yf);
        self.__setImageArea = self.__build_function("OrsayScansetImageArea",
                                                    [c_void_p, c_int, c_int, c_int, c_int, c_int, c_int, c_int],
                                                    c_bool)

        # bool SCAN_EXPORT OrsayScangetImageArea(void* o, int gene, int *sx, int *sy, int *xd, int *xf, int *yd, int *yf);
        self.__getImageArea = self.__build_function("OrsayScangetImageArea",
                                                    [c_void_p, c_int, POINTER(c_int), POINTER(c_int), POINTER(c_int),
                                                     POINTER(c_int), POINTER(c_int), POINTER(c_int)], bool)

        # double SCAN_EXPORT OrsayScangetPose(void* o, int gene);
        self.__getPose = self.__build_function("OrsayScangetPose", [c_void_p, c_int], c_double)

        # bool SCAN_EXPORT OrsayScansetPose(void* o, int gene, double time);
        self.__setPose = self.__build_function("OrsayScansetPose", [c_void_p, c_int, c_double],
                                               c_bool)

        # double SCAN_EXPORT OrsayScanSetRetourLigne(void* o, int gene, double rt);
        self.__SetRetourLigne = self.__build_function("OrsayScanSetRetourLigne",
                                                      [c_void_p, c_int, c_double],
                                                      c_double)
        # double SCAN_EXPORT OrsayScanGetRetourligne(void* o, int gene);
        self.__GetRetourLigne = self.__build_function("OrsayScanGetRetourLigne", [c_void_p, c_int],
                                                      c_double)

        # double SCAN_EXPORT OrsayScanGetImageTime(void* o, int gene);
        self.__GetImageTime = self.__build_function("OrsayScanGetImageTime", [c_void_p, c_int],
                                                    c_double)

        # bool SCAN_EXPORT OrsayScanSetInputs(void* o, int gene, int nb, int *inputs);
        self.__SetInputs = self.__build_function("OrsayScanSetInputs",
                                                 [c_void_p, c_int, c_int, POINTER(c_int)],
                                                 c_bool)

        # int SCAN_EXPORT OrsayScanGetInputs(void* o, int gene, int *inputs);
        self.__GetInputs = self.__build_function("OrsayScanGetInputs",
                                                 [c_void_p, c_int, POINTER(c_int)],
                                                 c_int)

        # void SCAN_EXPORT OrsayScanSetRotation(void* o, double angle);
        self.__SetRotation = self.__build_function("OrsayScanSetRotation", [c_void_p, c_double], None)

        # double SCAN_EXPORT OrsayScanGetRotation(void* o);
        self.__GetRotation = self.__build_function("OrsayScanGetRotation", [c_void_p], c_double)

        # bool SCAN_EXPORT OrsayScanStartImaging(void* o, short gene, short mode, short lineaverage);
        self.__StartImaging = self.__build_function("OrsayScanStartImaging",
                                                    [c_void_p, c_short, c_short, c_short],
                                                    c_bool)

        # bool SCAN_EXPORT OrsayScanStartSpim(void* o, short gene, short mode, short lineaverage, int nbspectraperpixel, bool sumpectra);
        self.__StartSpim = self.__build_function("OrsayScanStartSpim",
                                                 [c_void_p, c_short, c_short, c_short, c_int, c_bool], c_bool)

        #	bool OrsayStartTableImaging(void* o, int scannb, int mode, void *sp, unsigned long scan_count);
        self.__StartTableImaging = self.__build_function("OrsayStartTableImaging",
                                                         [c_void_p, c_int, c_int, c_void_p, c_uint64], c_bool)

        # bool SCAN_EXPORT OrsayScanStopImaging(void* o, int gene, bool cancel);
        self.__StopImaging = self.__build_function("OrsayScanStopImaging", [c_void_p, c_int, c_bool],
                                                   c_bool)

        # bool SCAN_EXPORT OrsayScanStopImagingA(void* o, int gene, bool immediate);
        self.__StopImagingA = self.__build_function("OrsayScanStopImagingA",
                                                    [c_void_p, c_int, c_bool], c_bool)

        # void SCAN_EXPORT OrsayScanSetImagingMode(void* o, int gene, int stripes);
        self.__SetImagingMode = self.__build_function("OrsayScanSetImagingMode",
                                                      [c_void_p, c_int, c_int],
                                                      None);

        # bool SCAN_EXPORT OrsayScanSetScanClock(void* o, int gene, int mode);
        self.__SetScanClock = self.__build_function("OrsayScanSetScanClock", [c_void_p, c_int, c_int],
                                                    c_bool)

        # unsigned long SCAN_EXPORT OrsayScanGetScansCount(void* o);
        self.__GetScansCount = self.__build_function("OrsayScanGetScansCount", [c_void_p], c_uint32)

        # void SCAN_EXPORT OrsayScanSetScale(void* o, int sortie, double vx, double vy);
        self.__SetScale = self.__build_function("OrsayScanSetScale",
                                                [c_void_p, c_int, c_double, c_double],
                                                None)

        # void SCAN_EXPORT OrsayScanSetImagingKind(void *o, int gene, int kind);
        self.__SetImagingKind = self.__build_function("OrsayScanSetImagingKind",
                                                      [c_void_p, c_int, c_int],
                                                      None)

        # int SCAN_EXPORT OrsayScanGetImagingKind(void *o, int gene);
        self.__GetImagingKind = self.__build_function("OrsayScanGetImagingKind", [c_void_p, c_int],
                                                      c_int)

        # double SCAN_EXPORT OrsayScanGetVideoOffset(void *o, int index);
        self.__GetVideoOffset = self.__build_function("OrsayScanGetVideoOffset", [c_void_p, c_int],
                                                      c_double)

        # void SCAN_EXPORT OrsayScanSetVideoOffset(void *o, int index, double value);
        self.__SetVideoOffset = self.__build_function("OrsayScanSetVideoOffset",
                                                      [c_void_p, c_int, c_double],
                                                      None)
        # bool SCAN_EXPORT OrsayScanSetFieldSize(self.orsayscan, double field);
        self.__SetFieldSize = self.__build_function("OrsayScanSetFieldSize", [c_void_p, c_double],
                                                    c_bool)

        # void SCAN_EXPORT OrsayScanRegisterDataLocker(void * o, void *(*LockScanDataPointer)(int gene, int *datatype, int *sx, int *sy, int *sz));
        self.__registerLocker = self.__build_function("OrsayScanRegisterDataLocker",
                                                      [c_void_p, LOCKERFUNC],
                                                      None)
        # void SCAN_EXPORT OrsayScanRegisterDataUnlocker(void *o, void(*UnLockScanDataPointer)(int gene, bool newdata));
        self.__registerUnlocker = self.__build_function("OrsayScanRegisterDataUnlocker",
                                                        [c_void_p, UNLOCKERFUNC],
                                                        None)
        self.__registerUnlockerA = self.__build_function("OrsayScanRegisterDataUnlockerA",
                                                         [c_void_p, UNLOCKERFUNCA], None)

        # bool SCAN_EXPORT OrsayScanSetProbeAt(self.orsayscan, int gene, int px, int py);
        self.__SetProbeAt = self.__build_function("OrsayScanSetProbeAt",
                                                  [c_void_p, c_int, c_int, c_int],
                                                  c_bool)

        # unsigned int SCAN_EXPORT GetRecordFrames(void *o, int gene);
        self.__GetRecordFrames = self.__build_function("GetRecordFrames", [c_void_p, c_int], c_uint32)

        # void SCAN_EXPORT SetRecordFrames(void *o, int gene, unsigned int nbframes);
        self.__SetRecordFrames = self.__build_function("SetRecordFrames", [c_void_p, c_int, c_uint32], None)

        # void SCAN_EXPORT OrsayScanSetEHT(self.orsayscan, double val);
        self.__SetEHT = self.__build_function("OrsayScanSetEHT", [c_void_p, c_double], None)

        # double SCAN_EXPORT OrsayScanGetEHT(self.orsayscan);
        self.__GetEHT = self.__build_function("OrsayScanGetEHT", [c_void_p], c_double)

        # double SCAN_EXPORT OrsayScanGetMaxFieldSize(self.orsayscan);
        self.__GetMaxFieldSize = self.__build_function("OrsayScanGetMaxFieldSize", [c_void_p],
                                                       c_double)

        # double SCAN_EXPORT OrsayScanGetFieldSize(self.orsayscan);
        self.__GetFieldSize = self.__build_function("OrsayScanGetFieldSize", [c_void_p], c_double)

        # double SCAN_EXPORT OrsayScanGetScanAngle(self.orsayscan, short *mirror);
        self.__GetScanAngle = self.__build_function("OrsayScanGetScanAngle", [c_void_p, c_short],
                                                    c_double)

        # bool SCAN_EXPORT OrsayScanSetFieldSize(self.orsayscan, double field);
        self.__SetFieldSize = self.__build_function("OrsayScanSetFieldSize", [c_void_p, c_double],
                                                    c_bool)

        # bool SCAN_EXPORT OrsayScanSetBottomBlanking(self.orsayscan, short mode, short source, double beamontime, bool risingedge, unsigned int nbpulses, double delay);
        self.__SetBottomBlanking = self.__build_function("OrsayScanSetBottomBlanking",
                                                         [c_void_p, c_short, c_short, c_double, c_bool, c_uint,
                                                          c_double],
                                                         c_bool)

        # bool SCAN_EXPORT OrsayScanSetTopBlanking(self.orsayscan, short mode, short source, double beamontime, bool risingedge, unsigned int nbpulses, double delay);
        self.__SetTopBlanking = self.__build_function("OrsayScanSetTopBlanking",
                                                      [c_void_p, c_short, c_short, c_double, c_bool, c_uint,
                                                       c_double], c_bool)

        #	bool SCAN_EXPORT OrsayScanSetTdcLine(void *o, short index, short mode, short source, double period, double ontime, bool risingedge, unsigned int nbpulses, double delay, bool filtered);
        self.__SetTdcLine = self.__build_function("OrsayScanSetTdcLine",
                                                  [c_void_p, c_short, c_short, c_short, c_double, c_double, c_bool,
                                                   c_uint32, c_double, c_bool], c_bool)

        # bool SCAN_EXPORT OrsayScanSetCameraSync(self.orsayscan, bool eels, int divider, double width, bool risingedge);
        self.__SetCameraSync = self.__build_function("OrsayScanSetCameraSync",
                                                     [c_void_p, c_bool, c_int, c_double, c_bool], c_bool)

        # void SCAN_EXPORT OrsayScanObjectiveStigmateur(self.orsayscan, double x, double y);
        self.__ObjectiveStigmateur = self.__build_function("OrsayScanObjectiveStigmateur",
                                                           [c_void_p, c_double, c_double], None)

        # void SCAN_EXPORT OrsayScanObjectiveStigmateurCentre(self.orsayscan, double xcx, double xcy, double ycx, double ycy);
        self.__ObjectiveStigmateurCentre = self.__build_function("OrsayScanObjectiveStigmateurCentre",
                                                                 [c_void_p, c_double, c_double, c_double, c_double],
                                                                 None)

        # void SCAN_EXPORT OrsayScanCondensorStigmateur(self.orsayscan, double x, double y);
        self.__CondensorStigmateur = self.__build_function("OrsayScanCondensorStigmateur",
                                                           [c_void_p, c_double, c_double], None)

        # void SCAN_EXPORT OrsayScanGrigson(self.orsayscan, double x1, double x2, double y1, double y2);
        self.__Grigson = self.__build_function("OrsayScanGrigson",
                                               [c_void_p, c_double, c_double, c_double, c_double], None)

        # void SCAN_EXPORT OrsayScanAlObjective(self.orsayscan, double x1, double x2, double y1, double y2);
        self.__AlObjective = self.__build_function("OrsayScanAlObjective",
                                                   [c_void_p, c_double, c_double, c_double, c_double], None)

        # void SCAN_EXPORT OrsayScanAlGun(self.orsayscan, double x1, double x2, double y1, double y2);
        self.__AlGun = self.__build_function("OrsayScanAlGun",
                                             [c_void_p, c_double, c_double, c_double, c_double],
                                             None)

        # void SCAN_EXPORT OrsayScanAlStigObjective(self.orsayscan, double x1, double x2, double y1, double y2);
        self.__AlStigObjective = self.__build_function("OrsayScanAlStigObjective",
                                                       [c_void_p, c_double, c_double, c_double, c_double], None)

        # void SCAN_EXPORT OrsayScanSetLaser(self.orsayscan, double frequency, int nbpulses, bool bottomblanking, short sync);
        self.__SetLaser = self.__build_function("OrsayScanSetLaser",
                                                [c_void_p, c_double, c_int, c_bool, c_short],
                                                None)

        # void SCAN_EXPORT OrsayScanStartLaser(self.orsayscan, int mode);
        self.__StartLaser = self.__build_function("OrsayScanStartLaser", [c_void_p, c_int], None)

        # void SCAN_EXPORT OrsayScanStartLaserA(self.orsayscan, int mode, short source);
        self.__StartLaserA = self.__build_function("OrsayScanStartLaserA", [c_void_p, c_int, c_short],
                                                   None)

        # void SCAN_EXPORT OrsayScanCancelLaser(self.orsayscan);
        self.__CancelLaser = self.__build_function("OrsayScanCancelLaser", [c_void_p], None)

        # int SCAN_EXPORT OrsayScanGetLaserCount(self.orsayscan);
        self.__GetLaserCount = self.__build_function("OrsayScanGetLaserCount", [c_void_p], c_int)

        #	double SCAN_EXPORT GetClockSimulationTime(void *o, int gene);
        self.__GetClockSimulationTime = self.__build_function("GetClockSimulationTime",
                                                              [c_void_p, c_int],
                                                              c_double)
        #	void SCAN_EXPORT SetClockSimulationTime(void *o, int gene, double dt);
        self.__SetClockSimulationTime = self.__build_function("SetClockSimulationTime",
                                                              [c_void_p, c_int, c_double],
                                                              None)

        # double SCAN_EXPORT OrsayScanGetPMT(self.orsayscan, int index);
        self.__GetPMT = self.__build_function("OrsayScanGetPMT", [c_void_p, c_int], c_double)

        # void SCAN_EXPORT OrsayScanSetPMT(self.orsayscan, int index, double value);
        self.__SetPMT = self.__build_function("OrsayScanSetPMT", [c_void_p, c_int, c_double], None)

        # int SCAN_EXPORT OrsayScanCountPMTs (self.orsayscan)
        self.__CountPMTs = self.__build_function("OrsayScanCountPMTs", [c_void_p], c_int)

        # bool SCAN_EXPORT OrsayScanGetPMTLimits(self.orsayscan, int index, double &vmin, double & vmax)
        self.__GetPMTLimits = self.__build_function("OrsayScanGetPMTLimits",
                                                    [c_void_p, c_int, c_double, c_double], c_bool)

        #	void SCAN_EXPORT OrsayScanSetVSMParameters(int kind, double offset, double gain);
        self.__SetVSMParameters = self.__build_function("OrsayScanSetVSMParameters",
                                                        [c_void_p, c_int, c_double, c_double], None)

        #	double SCAN_EXPORT OrsayScanGetVSM(void *o);
        self.__GetVSM = self.__build_function("OrsayScanGetVSM", [c_void_p], c_double)

        #	void SCAN_EXPORT OrsayScanSetVSM(void *o, double value);
        self.__SetVSM = self.__build_function("OrsayScanSetVSM", [c_void_p, c_double], None)

        # 	void SCAN_EXPORT OrsayScanSetRemote(void *o, bool active);
        self.__SetRemote = self.__build_function("OrsayScanSetRemote", [c_void_p, c_bool], None)
        # 	bool SCAN_EXPORT OrsayScanGetRemote(void *o);
        self.__GetRemote = self.__build_function("OrsayScanGetRemote", [c_void_p], c_bool)

        # int  SCAN_EXPORT OrsayScanGetInputLineIndex(const char* name, bool litteral);
        self.__GetInputLineIndex = self.__build_function("OrsayScanGetInputLineIndex",
                                                         [c_void_p, c_char_p, c_bool], c_int)
        # int  SCAN_EXPORT OrsayScanGetOutputLineIndex(const char* name, bool litteral);
        self.__GetOutputLineIndex = self.__build_function("OrsayScanGetOutputLineIndex",
                                                          [c_void_p, c_char_p, c_bool], c_int)
        # bool  SCAN_EXPORT OrsayScanSetOutputLine(short index, short mode, short source, double period = 0, double ontime = 1e-6, bool risingedge = true, unsigned int nbpulses = 0, double delay = 0, bool filtered = false);
        self.__SetOutputLine = self.__build_function("OrsayScanSetOutputLine",
                                                     [c_void_p, c_short, c_short, c_short, c_double, c_double, c_bool,
                                                      c_uint32, c_double,
                                                      c_bool], c_bool)
        #	void SCAN_EXPORT OrsayScanEnableLines(void* o, short mask);
        self.__EnableLines = self.__build_function("OrsayScanEnableLines", [c_void_p, c_short], None)

        #   void SCAN_EXPORT OrsayScanEnableLines(void* o, short mask);
        self.__DisableLines = self.__build_function("OrsayScanDisableLines", [c_void_p, c_short], None)

        # void SCAN_EXPORT SetPulseLevel(void *o, int voie, double value);
        self.__SetPulseLevel = self.__build_function("SetPulseLevel", [c_void_p, c_int, c_double],
                                                     None)
        # void SCAN_EXPORT SetPulseDivider(void *o, int n);
        self.__SetPulseDivider = self.__build_function("SetPulseDivider", [c_void_p, c_int], None)

        # void SCAN_EXPORT SetLowMag(void* o, bool low);
        self.__SetLowMag = self.__build_function("SetLowMag", [c_void_p, c_bool], None)
        # bool SCAN_EXPORT IsLowMag(void* o);
        self.__IsLowMag = self.__build_function("IsLowMag", [c_void_p], c_bool)

        #	void SCAN_EXPORT SetPixelDelay(void* o, int value);
        self.__SetPixelDelay = self.__build_function("SetPixelDelay", [c_void_p, c_int], None)
        #   int SCAN_EXPORT GetPixelDelay(void* o);
        self.__GetPixelDelay = self.__build_function("GetPixelDelay", [c_void_p], c_int)

    def __init_scan_pattern_interface(self):
        self.scan_pattern = scan_pattern.ScanPattern(self.__scan_dll)

    def __init_drift_cancellation_interface(self):
        self.drift_cancellation = drift_cancellation.drift_cancellation(self.__scan_dll)

    def __init__(self, gene: int, scandllobject=0, vg=False, efm03 = False) -> None:
        self.__initialize()
        #
        # Should read initial value from configuration file
        #

        self.gene = gene
        cproduct = c_short()
        crevision = c_short()
        cserialnumber = c_short()
        cmajor = c_short()
        cminor = c_short()
        self.__drift_roi_correction = False
        if (gene < 2):
            library_folder = self.__python_folder.encode("utf-8")
            self.orsayscan = self.__InitEx(not vg, library_folder, efm03)
        if (gene > 1):
            self.orsayscan = scandllobject
        self.__getVersion(self.orsayscan, byref(cproduct), byref(crevision), byref(cserialnumber),
                                   byref(cmajor), byref(cminor))
        self._product = cproduct.value
        self._revision = crevision.value
        self._serialnumber = cserialnumber.value
        self._major = cmajor.value
        self._minor = cminor.value
        self.inputs = list(self.__input_properties(i) for i in range(self.getInputsCount()))
        if self._major < 5:
            raise AttributeError("No device connected")
        if (gene == 1):
            self.__load_drift_tube()
        self.__init_scan_pattern_interface()
        self.__init_drift_cancellation_interface()

    def __load_drift_tube(self) -> None:
        self.__drift_tube_config_file = os.environ['ALLUSERSPROFILE'] + "\\Nion\\Nion Swift\\Drift_tube.json"
        try:
            with open(self.__drift_tube_config_file) as f:
                self.__drift_tube = json_load(f)
                # check that kind key is present.
                if _("kind") not in self.__drift_tube:
                    self.__drift_tube["kind"] = -1;
        except Exception as e:
            self.__drift_tube = {"kind": -1, "offset": 0.0, "gain": 1.0 / 10.0,
                                 "range": {"min": -10.0, "max": 10.0}, "value": 0.0}
            try:
                with open(self.__drift_tube_config_file, "w") as fp:
                    json_dump(self.__drift_tube, fp, skipkeys=True, indent=4)
            except Exception as ex:
                print("error saving vsm calibrations")
            finally:
                pass
        finally:
            pass
        self.drift_tube_calibration = self.__drift_tube

    def close(self) -> None:
        self.__Close(self.orsayscan)
        self.orsaycamera = None

    def __verifyunsigned32bit(self, value: int) -> None:
        """
        Check if value is in range 0 <= value <= 0xffffffff
        """
        if (value < 0 or value > 0xffffffff):
            raise AttributeError("Argument out of range (must be 32bit unsigned).")

    def __verifysigned32bit(self, value: int) -> None:
        """
        Check if value is in range 0 <= value <= 0xffffffff
        """
        if (value < 0x8000000 or value > 0x7fffffff):
            raise AttributeError("Argument out of range (must be 32bit signed).")

    def __verifypositiveint(self, value: int) -> None:
        """
        Check if value is in range 0 <= value <= 0xffffffff
        """
        if (value < 0 or value > 0x7fffffff):
            raise AttributeError("Argument out of range (must be positive 32bit signed).")

    def __verifystrictlypositiveint(self, value: int) -> None:
        """
        Check if value is in range 0 < value <= 0xffffffff
        """
        if (value < 0 or value > 0x7fffffff):
            raise AttributeError("Argument out of range (must be positive 32bit signed).")

    def getInputsCount(self) -> int:
        """
        Donne le nombre d'entrées vidéo actives
        """
        return self.__getInputsCount(self.orsayscan)

    @property
    def inputs_count(self) -> int:
        """
        Donne le nombre d'entrées vidéo actives
        """
        return self.__getInputsCount(self.orsayscan)

    def __input_properties(self, input: int) -> scan_input:
        """
        Lit les propriétés de l'entrée vidéo
        Retourne la calsse scan_input.
        """
        unipolar = c_bool()
        offset = c_double()
        buffer = create_string_buffer(b'\000' * 100)
        res = self.__GetInputProperties(self.orsayscan, input, byref(unipolar), byref(offset), buffer)
        return scan_input(buffer.value.decode("utf-8"), input, unipolar.value, offset.value, False).as_dict()

    def getInputProperties(self, input: int) -> (int, float, str, int):
        """
        Lit les propriétés de l'entrée vidéo
        Retourne 3 valeurs: bool vrai si unipolaire, double offset, string nom, index de l'entrée.
        """
        unipolar = c_bool()
        offset = c_double()
        buffer = create_string_buffer(b'\000' * 100)
        res = self.__GetInputProperties(self.orsayscan, input, byref(unipolar), byref(offset), buffer)
        return unipolar.value, offset.value, buffer.value.decode("utf-8"), input

    def setInputProperties(self, input: int, unipolar: bool, offset: float) -> bool:
        """
        change les propriétés de l'entrée vidéo
        Pour le moment, seul l'offset est utilisé.
        """
        res = self.__SetInputProperties(self.orsayscan, input, offset)
        if (not res):
            raise Exception("Failed to set orsayscan input properties")
        return res

    def GetImageTime(self) -> float:
        """
        Donne le temps effectif de la durée de balayage d'une image
        """
        return self.__GetImageTime(self.orsayscan, self.gene)

    def SetInputs(self, inputs: []) -> bool:
        """
        Choisit les entrées à lire.
        A cause d'une restriction hardware, les valeurs possibles sont 1, 2, 4, 6, 8
        """
        inputarray = (c_int * len(inputs))()
        k = 0
        while (k < len(inputs)):
            inputarray[k] = inputs[k]
            k = k + 1
        return self.__SetInputs(self.orsayscan, self.gene, len(inputarray), inputarray)

    def GetInputs(self) -> (int, []):
        """
        Donne la liste des entrées utilisées
        """
        inputarray = (c_int * 20)()
        nbinputs = self.__GetInputs(self.orsayscan, self.gene, inputarray)
        inputs = []
        for inp in range(0, nbinputs):
            inputs.append(inputarray[inp])
        return nbinputs, inputs

    def setImageSize(self, sizex: int, sizey: int) -> bool:
        """
        Définit la taille de l'image en pixels
        Les limites de dimension sont 1 et 8192
        """
        res = self.__setImageSize(self.orsayscan, self.gene, sizex, sizey)
        if (not res):
            raise Exception("Failed to set orsayscan image size")

    def getImageSize(self) -> (int, int):
        """
        Donne la taille de l'image
        *** il est impératif que le tableau passé à la callback ait cette taille
            multipliée par le nombre d'entrées, multipliée par le paramètre lineaveragng ***
        """
        sx = c_int()
        sy = c_int()
        res = self.__getImageSize(self.orsayscan, self.gene, byref(sx), byref(sy))
        if (not res):
            raise Exception("Failed to get orsayscan image size")
        return int(sx.value), int(sy.value)

    @property
    def Image_area(self):
        return self.__scan_area

    @Image_area.setter
    def Image_area(self, value):
        self.__scan_area = value
        self.setImageArea(self.__scan_area[0], self.__scan_area[1], self.__scan_area[2], self.__scan_area[3],
                          self.__scan_area[4], self.__scan_area[5])

    def setImageArea(self, sizex: int, sizey: int, startx: int, endx: int, starty: int, endy: int) -> bool:
        """
        Définit une aire pour le balayage.
        Définit une aire pour le balayage.
        sizex, sizey taille de l'image complète
        startx, endx début et fin de la largeur du rectangle
        starty, endy début et fin de la hauteur.
        """
        #        self.__verifyStrictlyPositiveInt(sizex)
        #        self.__verifyStrictlyPositiveInt(sizey)
        res = self.__setImageArea(self.orsayscan, self.gene, sizex, sizey, startx, endx, starty, endy)
        return res

    def getImageArea(self) -> (bool, int, int, int, int, int, int):
        """
        Donne l'aire réduite utilisée,
        retourne les paramètres donnés à la fonction setImageArea ou ceux les plus proches valides.
        """
        sx, sy, stx, ex, sty, ey = c_int(), c_int(), c_int(), c_int(), c_int(), c_int()
        res = self.__getImageArea(self.orsayscan, self.gene, byref(sx), byref(sy), byref(stx), byref(ex),
                                           byref(sty), byref(ey))
        return res, int(sx.value), int(sy.value), int(stx.value), int(ex.value), int(sty.value), int(ey.value)

    @property
    def pixelTime(self) -> float:
        """
        Donne le temps par pixel
        """
        return self.__getPose(self.orsayscan, self.gene)

    @pixelTime.setter
    def pixelTime(self, value: float):
        """
        Définit le temps par pixel
        """
        self.__setPose(self.orsayscan, self.gene, value)

    @property
    def record_frames(self) -> int:
        return self.__GetRecordFrames(self.orsayscan, self.gene)

    @record_frames.setter
    def record_frames(self, value: int) -> None:
        self.__SetRecordFrames(self.orsayscan, self.gene, value)

    #
    #   Callback qui sera appelée lors d'arrivée de nouvelles données
    #
    def registerLocker(self, fn: LOCKERFUNC) -> None:
        """
        Définit la fonction callback appelée lorsque de nouvelles données sont présentes
        Elle a pour but de passer un tableau image sa dimension et son type de données
        On ne doit détruire cet objet avant l'appel d'une fonction unlock
        Voir programme demo.
        """
        self.__registerLocker(self.orsayscan, fn)

    def registerUnlocker(self, fn: UNLOCKERFUNC) -> None:
        """
        Definit la fonction appelée à la fin du transfert de données.
        recoit newdata vrai si de nouvelles données sont effectivement là.
        Utiliser de préférence la fonction registerUnlockerA plus riche en informations sur le flux de données
        voir programe demo
        """
        self.__registerUnlocker(self.orsayscan, fn)

    def registerUnlockerA(self, fn: UNLOCKERFUNCA) -> None:
        """
        Definit la fonction appelée à la fin du transfert de données.
        reçoit newdata, le numéro de séquence de l'image en cours, rect: les coordonnées du rect où les données ont été modifiées.
        voir programe demo
        """
        self.__registerUnlockerA(self.orsayscan, fn)

    def startSpim(self, mode: int, linesaveraging: int, Nspectra=1, save2D=False) -> bool:
        """
        Démarre l'acquitisition de l'image.
        mode: --- expliqué plus tard ---
        lineaveraging: nombre de lignes à faire avant de passer à la ligne suivante.
        retourne vrai si l'acquisition a eu lieu.
        """
        return self.__StartSpim(self.orsayscan, self.gene, mode, linesaveraging, Nspectra, save2D)

    def setScanClock(self, trigger_input=0) -> bool:
        """
        set the input line for starting the next pixel in the STEM imaging (pin 9 and 5 on subD9)
        Parameters
        ----------
        trigger_input: 0 for pin 9, 1 for pin 5, 2 for CL ready, 3 for In3, 4 for EELS ready
        Returns
        -------
        """
        return self.__SetScanClock(self.orsayscan, self.gene, trigger_input)

    def startImaging(self, mode: int, linesaveraging: int) -> bool:
        """
        Démarre l'acquisition de l'image.
        mode: --- expliqué plus tard ---
        lineaveraging: nombre de lignes à faire avant de passer à la ligne suivante.
        retourne vrai si l'acquisition a eu lieu.
        """
        return self.__StartImaging(self.orsayscan, self.gene, mode, linesaveraging)

    def stopImaging(self, cancel: bool) -> bool:
        """
        Arrete l'acquisition d'images
        cancel vrai => immédiat,  faux => à la fin du scan de l'image en cours
        """
        return self.__StopImaging(self.orsayscan, self.gene, cancel)

    def startTableImaging(self, mode, pattern, scan_count):
        """
        Scan using a pattern in memory.
        Image can be done while scanning
        gene: scan unit 1 or 2
        mode: display control , stripes or full image at once
        pattern: scan pattern used
        scan_count: number of scans to do, from 1 to 2**64 - 1
        return True when everything went fine.
        """
        return self.__StartTableImaging(self.orsayscan, self.gene, mode, pattern, scan_count)

    def getScanCount(self) -> int:
        """
        Donne le nombe de balayages déjà faits
        """
        return self.__GetScansCount(self.orsayscan)

    def setScanRotation(self, angle: float) -> None:
        """
        Définit l'angle de rotation du balayage de l'image
        """
        self.__SetRotation(self.orsayscan, angle)

    def getScanRotation(self) -> float:
        """
        Relit la valeur de l'angle de rotation du balayage de l'image
        """
        return self.__GetRotation(self.orsayscan)

    def setScanScale(self, plug, xamp: float, yamp: float) -> None:
        """
        Ajuste la taille des signaux analogiques de balayage valeur >0 et inf"rieure à 1.
        """
        self.__SetScale(self.orsayscan, plug, xamp, yamp)

    def getImagingKind(self) -> int:
        kind = self.__GetImagingKind(self.orsayscan, self.gene)
        return kind

    def setVideoOffset(self, inp: int, offset: float) -> None:
        """
        Définit l'offset analogique à ajouter au signal d'entrée afin d'avoir une valeur 0 pour 0 volts
        En principe, c'est un réglage et pour une machine cela ne devrait pas bouger beaucoup
        """
        self.__SetVideoOffset(self.orsayscan, inp, offset)

    def getVideoOffset(self, inp: int) -> float:
        """
        Donne la valeur de l'offset vidéo
        """
        return self.__GetVideoOffset(self.orsayscan, inp)

    def SetProbeAt(self, px: int, py: int) -> bool:
        # bool SCAN_EXPORT OrsayScanSetProbeAt(self.orsayscan, int gene, int px, int py);
        return self.__SetProbeAt(self.orsayscan, self.gene, px, py)

    # void SCAN_EXPORT OrsayScanSetEHT(self.orsayscan, double val);
    def SetEHT(self, val: float) -> None:
        self.__SetEHT(self.orsayscan, val)

    def GetEHT(self) -> float:
        return self.__GetEHT(self.orsayscan)

    # double SCAN_EXPORT OrsayScanGetMaxFieldSize(self.orsayscan);
    def GetMaxFieldSize(self) -> float:
        return self.__GetMaxFieldSize(self.orsayscan)

    # double SCAN_EXPORT OrsayScanGetFieldSize(self.orsayscan);
    def GetFieldSize(self) -> float:
        return self.__GetFieldSize(self.orsayscan)

    # double SCAN_EXPORT OrsayScanGetScanAngle(self.orsayscan, short *mirror);
    def GetScanAngle(self, mirror) -> float:
        return self.__GetScanAngle(self.orsayscan, mirror)

    # bool SCAN_EXPORT OrsayScanSetFieldSize(self.orsayscan, double field);
    def SetFieldSize(self, field: float) -> bool:
        return self.__SetFieldSize(self.orsayscan, field)

    # bool SCAN_EXPORT OrsayScanSetBottomBlanking(self.orsayscan, short mode, short source, double beamontime, bool risingedge, unsigned int nbpulses, double delay);
    def SetBottomBlanking(self, mode: int, source: int, beamontime=0, risingedge=True, nbpulses=0, delay=0) -> bool:
        """ Définit le blanker avant l'échantillon sur un VG/Nion
            mode : 0 blanker off, 1 blanker On, 2 controlled by source,
            3 controlled by source but with locally defined time (beamontime parametre)
            source : to be choosen based on configuration file (eels
            camera readout, cl camera readout, laser pulse, ...)
            beamontime : with of the Blanker on signal, for instance CCD
            vertical transfer time, laser pulse width, ...
            risingedge : choose the edge that triggers the beamontime.
            nbpulses : number of pulses required a signal is generated
            (used to sync slave cameras)
            delay : delay used to generate the beamon signal after the
            trigger. if nbpulses != 0, this delay is incremented nbpulses times.
            (very specific application not tested yet).
        """
        return self.__SetBottomBlanking(self.orsayscan, mode, source, beamontime, risingedge, nbpulses, delay)

    # bool SCAN_EXPORT OrsayScanSetTopBlanking(self.orsayscan, short mode, short source, double beamontime, bool risingedge, unsigned int nbpulses, double delay);
    def SetTopBlanking(self, mode: int, source: int, beamontime=0, risingedge=True, nbpulses=0, delay=0) -> bool:
        """ Définit le blanker après l'échantillon sur un VG/Nion
            mode : 0 blanker off, 1 blanker On, 2 controlled by source, 3 controlled by source but with locally defined time (beamontime parametre)
            source : to be choosen based on configuration file (eels camera readout, cl camera readout, laser pulse, ...)
            beamontime : with of the Blanker on signal, for instance CCD vertical transfer time, laser pulse width, ...
            risingedge : choose the edge that triggers the beamontime.
            nbpulses : number of pulses required a signal is generated (used to sync slave cameras)
            delay : delay used to generate the beamon signal after the trigger. if nbpulses != 0, this delay is incremented nbpulses times.
            (very specific application not tested yet).
        """
        return self.__SetTopBlanking(self.orsayscan, mode, source, beamontime, risingedge, nbpulses, delay)

    #	bool SCAN_EXPORT OrsayScanSetTdcLine(void *o, short index, short mode, short source, double period, double ontime, bool risingedge, unsigned int nbpulses, double delay);
    def SetTdcLine(self, line: int, mode: int, source: int, period: float, on_time: float, rising_edge: bool,
                   nb_pulses: int, delay: float, filtered=False) -> bool:
        """ defines how "Tdc" (CheeTah) works. Output Tdc are defined in scan.xml. If not does nothing.
            line : 0 ou 1
            mode : 0 => 0
                 : 1 => 1
                 : 2 copy of source
                 : 3 copy of source, fix output with (on_time, larger or smaller) if delay > 0 output is delayed
                 : 4
                 : 5
                 : 6
                 : 7 internal generator nbpulses = 0 continuous, nbpulses > 0 limited number of pulses
            source : from 0 to 4 input IO of the box
                   : 5 start of pixel clock (imaging mode) or bit 30 of the scan list.
                   : 6 end of pixel clock
                   : 7 blanking, falling edge means start of the line
                   : from 8 to 15 output line of the box. Should be different of Tdc Line of course, laser line is nice.
            period : frequency of internal generator (20 ns step)
            ontime : ontime (2.5 ns step)
            risingedge : edge used for mode > 2
            delay : when > 0 output is delayed
            filtered : if set, input line must high 4 clock periods at least in order to be seen.
        """
        return self.__SetTdcLine(self.orsayscan, line, mode, source, period, on_time, rising_edge, nb_pulses,
                                          delay, filtered)

    # bool SCAN_EXPORT OrsayScanSetCameraSync(self.orsayscan, bool eels, int divider, double width, bool risingedge);
    def SetCameraSync(self, eels: bool, divider: int, width: float, risingedge: bool) -> bool:
        """ Définit le mode de travail de la camera, par défaut la camera eels est maître
            eels: True => master, False => Slave
            divider: si mode slave, nombre d'impulsions pour avoir un trigger
            width: Largeur de l'impulsion
            risingedge: front utiliser pour compter l'impulsion.
        """
        return self.__SetCameraSync(self.orsayscan, eels, divider, width, risingedge)

    #
    #   Fonctions spécifique au VG
    #

    # void SCAN_EXPORT OrsayScanObjectiveStigmateur(self.orsayscan, double x, double y);
    def ObjectiveStigmateur(self, x: float, y: float) -> None:
        """ Définit le stigmateur objectif (électrostatique) """
        self.__ObjectiveStigmateur(self.orsayscan, x, y)

    # void SCAN_EXPORT OrsayScanObjectiveStigmateurCentre(self.orsayscan, double xcx, double xcy, double ycx, double ycy);
    def ObjectiveStigmateurCentre(self, xcx: float, xcy: float, ycx: float, ycy: float) -> None:
        """ Définit le centre du stigmateur objectif """
        self.__ObjectiveStigmateurCentre(self.orsayscan, xcx, xcy, ycx, ycy)

    # void SCAN_EXPORT OrsayScanCondensorStigmateur(self.orsayscan, double x, double y);
    def CondensorStigmateur(self, x: float, y: float) -> None:
        """ Définit le stigmateur condensuer (magnétique) """
        self.__CondensorStigmateur(self.orsayscan, x, y)

    # void SCAN_EXPORT OrsayScanGrigson(self.orsayscan, double x1, double x2, double y1, double y2);
    def Grigson(self, x1: float, x2: float, y1: float, y2: float) -> None:
        """ Définit le courant Grigson """
        self.__Grigson(self.orsayscan, x1, x2, y1, y2)

    # void SCAN_EXPORT OrsayScanAlObjective(self.orsayscan, double x1, double x2, double y1, double y2);
    def AlObjective(self, x1: float, x2: float, y1: float, y2: float) -> None:
        """ Aligne l'objectif """
        self.__AlObjective(self.orsayscan, x1, x2, y1, y2)

    # void SCAN_EXPORT OrsayScanAlGun(self.orsayscan, double x1, double x2, double y1, double y2);
    def AlGun(self, x1: float, x2: float, y1: float, y2: float) -> None:
        """ Aligne le canon """
        self.__AlGun(self.orsayscan, x1, x2, y1, y2)

    # void SCAN_EXPORT OrsayScanAlStigObjective(self.orsayscan, double x1, double x2, double y1, double y2);
    def AlStigObjective(self, x1: float, x2: float, y1: float, y2: float) -> None:
        """ Aligne le stigmateur canon(?) """
        self.__AlStigObjective(self.orsayscan, x1, x2, y1, y2)

    # void SCAN_EXPORT OrsayScanSetLaser(self.orsayscan, double frequency, int nbpulses, bool bottomblanking, short sync);
    def SetLaser(self, frequency: float, nbpulses: int, bottomblanking: bool, sync: int) -> None:
        """ définit le mode de travail du laser
            frequency: frequence des impulsions
            nbpulses: nombre total d'impulsions sur le prochain tir
            bottomblanking: True => utilisé
            sync: < 0 pas utilisé, >= 0 utilise l'entrée choisie pour déclencher l'impulsion
        """
        self.__SetLaser(self.orsayscan, frequency, nbpulses, bottomblanking, sync)

    # void SCAN_EXPORT OrsayScanStartLaser(self.orsayscan, int mode);
    def StartLaser(self, mode: int, source=-1) -> None:
        """
        Démarre le laser
        when mode = 7 => internal generator with frequency defined by SetLaser function
        when mode = 3 => source is taken as trigger, for instance 5 for pixel clock.
        """
        if source == -1:
            self.__StartLaser(self.orsayscan, mode)
        else:
            self.__StartLaserA(self.orsayscan, mode, source)

    # void SCAN_EXPORT OrsayScanCancelLaser(self.orsayscan);
    def CancelLaser(self) -> None:
        """ arrete le laser """
        self.__CancelLaser(self.orsayscan)

    # int SCAN_EXPORT OrsayScanGetLaserCount(self.orsayscan);
    def GetLaserCount(self) -> int:
        """ donne le nombre d'impulsions déjà faites """
        return self.__GetLaserCount(self.orsayscan)

    # double SCAN_EXPORT OrsayScanGetPMT(self.orsayscan, int index);
    def GetPMT(self, index: int) -> float:
        return self.__GetPMT(self.orsayscan, index)

    # void SCAN_EXPORT OrsayScanSetPMT(self.orsayscan, int index, double value);
    def SetPMT(self, index: int, value: float) -> None:
        self.__SetPMT(self.orsayscan, index, value)

    # int SCAN_EXPORT OrsayScanCountPMTs (self.orsayscan)
    def CountPMTs(self) -> int:
        return self.__CountPMTs(self.orsayscan)

    # bool SCAN_EXPORT OrsayScanGetPMTLimits(self.orsayscan, int index, double &vmin, double & vmax)
    def GetPMTLimits(self, index: int) -> (bool, float, float):
        vmin = 0.0
        vmax = 1000.0
        res = self.__GetPMTLimits(self.orsayscan, index, vmin, vmax)
        return res, vmin, vmax

    ### END YVES ###
    @property
    def clock_simulation_time(self) -> float:
        """
        Donne le temps par pixel pour le spectre image
        Si == 0: pas de simulation de d'horloge camera,
        Si > 0: l'horloge caméra est générée en interne par scanner.
        """
        return self.__GetClockSimulationTime(self.orsayscan, self.gene)

    @clock_simulation_time.setter
    def clock_simulation_time(self, value: float):
        """
        Donne le temps par pixel pour le spectre image
        Si 0, pas de simulation d'horloge camera.
        """
        self.__SetClockSimulationTime(self.orsayscan, self.gene, value)

    def SetFlybackTime(self, flyback: float) -> float:
        return self.__SetRetourLigne(self.orsayscan, self.gene, flyback)

    def GetFlybackTime(self) -> float:
        return self.__GetRetourLigne(self.orsayscan, self.gene)

    @property
    def drift_tube_calibration(self) -> dict:
        return [self.__drift_tube["offset"], self.__drift_tube["gain"], self.__drift_tube["kind"]]

    @drift_tube_calibration.setter
    def drift_tube_calibration(self, calib: dict):
        self.__drift_tube["offset"] = calib["offset"]
        self.__drift_tube["gain"] = calib["gain"]
        # calculate new range
        self.__drift_tube["range"]["min"] = -1 / self.__drift_tube["gain"] + self.__drift_tube["offset"]
        self.__drift_tube["range"]["max"] = 1 / self.__drift_tube["gain"] + self.__drift_tube["offset"]
        if "kind" in calib:
            self.__drift_tube["kind"] = calib["kind"]
            if calib["kind"] >= 0:
                self.__SetVSMParameters(calib["kind"], self.__drift_tube["offset"], calib["gain"])
        # refresh value with new calibrations
        self.drift_tube = self.__drift_tube["value"]
        # save calibration values
        try:
            with open(self.__drift_tube_config_file, "w") as fp:
                json_dump(self.__drift_tube, fp, skipkeys=True, indent=4)
        except Exception as ex:
            print("error saving vsm calibrations")

    @property
    def drift_tube(self) -> float:
        """
        read back the current value of drift tube voltage, ie energy shift
        """
        return self.__drift_tube["value"]

    @drift_tube.setter
    def drift_tube(self, value: float) -> None:
        """
        Set the drift tube voltage
        """
        if (value > self.__drift_tube["range"]["max"]):
            value = self.__drift_tube["range"]["max"]
        else:
            if (value < self.__drift_tube["range"]["min"]):
                value = self.__drift_tube["range"]["min"]
        self.__drift_tube["value"] = value
        if self.__drift_tube["kind"] < 0:
            dac_value = 32767 * (value - self.__drift_tube["offset"]) * self.__drift_tube["gain"]
            self.__SetVSM(self.orsayscan, dac_value)
        else:
            self.__SetVSM(self.orsayscan, value)

    @property
    def remote(self) -> bool:
        return self.__GetRemote(self.orsayscan)

    @remote.setter
    def remote(self, value: bool):
        self.__SetRemote(self.orsayscan, value)

    def get_input_line_index(self, name: str, full_word: bool) -> int:
        return self.__GetInputLineIndex(self.orsayscan, name, full_word)

    def get_output_line_index(self, name: str, full_word: bool) -> int:
        return self.__GetOutputLineIndex(self.orsayscan, name, full_word)

    def set_output_line_a(self, index: int, mode: int, source=0, period=0.0, on_time=1e-6, rising_edge=True,
                          nb_pulses=0, delay=0.0, filtered=False) -> bool:
        """
        line : 0 ou 1
        mode : 0 => 0
             : 1 => 1
             : 2 copy of source
             : 3 copy of source, fix output with (on_time, larger or smaller) if delay > 0 output is delayed
             : 4
             : 5
             : 6
             : 7 internal generator nbpulses = 0 continuous, nbpulses > 0 limited number of pulses
        source : from 0 to 4 input IO of the box
               : 5 start of pixel clock (imaging mode) or bit 30 of the scan list.
               : 6 end of pixel clock
               : 7 blanking, falling edge means start of the line
               : from 8 to 15 output line of the box. Should be different of Tdc Line of course, laser line is nice.
        period : frequency of internal generator (20 ns step)
        on_time : ontime (2.5 ns step)
        rising_edge : edge used for mode > 2
        delay : when > 0 output is delayed
        filtered : if set, input line must high 4 clock periods at least in order to be seen.
        """
        return self.__SetOutputLine(self.orsayscan, index, mode, source, period, on_time, rising_edge,
                                             nb_pulses, delay, filtered)

    def enable_lines(self, lines:int) -> None:
        """
        Activate lines in mode > 2
        1 bit per line
        """
        return self.__EnableLines(self.orsayscan, lines)

    def disable_lines(self, lines: int) -> None:
        """
        Dectivate lines in mode > 2
        1 bit per line
        """
        return self.__DisableLines(self.orsayscan, lines)

    def set_output_line(self, properties: dict()):
        """
        line : 0 ou 1
        mode : 0 => 0
             : 1 => 1
             : 2 copy of source
             : 3 copy of source, fix output with (on_time, larger or smaller) if delay > 0 output is delayed
             : 4
             : 5
             : 6
             : 7 internal generator nb_pulses = 0 continuous, nb_pulses > 0 limited number of pulses
        source : from 0 to 4 input IO of the box
               : 5 start of pixel clock (imaging mode) or bit 30 of the scan list.
               : 6 end of pixel clock
               : 7 blanking, falling edge means start of the line
               : from 8 to 15 output line of the box. Should be different of Tdc Line of course, laser line is nice.
        period : frequency of internal generator (20 ns step)
        on_time : ontime (2.5 ns step)
        rising_edge : edge used for mode > 2
        delay : when > 0 output is delayed
        filtered : if set, input line must high 4 clock periods at least in order to be seen.
        """
        index = properties["index"]
        mode = properties["mode"]
        source = properties["mode"]
        period = properties["period"]
        on_time = properties["on_time"]
        rising_edge = properties["rising_edge"]
        nb_pulses = properties["nb_pulses"]
        delay = properties["delay"]
        filtered = properties["filtered"]
        return self.__SetOutputLine(self.orsayscan, index, mode, source, period, on_time, rising_edge,
                                             nb_pulses, delay, filtered)

    @property
    def pixel_delay(self) -> int:
        return self.__GetPixelDelay(self.orsayscan)

    @pixel_delay.setter
    def pixel_delay(self, value: int) -> None:
        self.__SetPixelDelay(self.orsayscan, value)