import os, logging, json, sys
from nion.utils import Registry

def InstrumentDictSetter(type, name, value):
    main_controller = Registry.get_component("stem_controller")
    if main_controller:
        try:
            main_controller.SetValDetailed(type, name, value)
        except AttributeError:
            it_exists, _ = main_controller.TryGetVal(name)
            if it_exists:
                main_controller.SetVal(name, value)

class FileManager:
    def __init__(self, filename):
        self.filename = filename
        if sys.platform.startswith('win'):
            self.abs_path = os.path.abspath('C:\\ProgramData\\Microscope\\' + filename + '.json')
        else:
            self.abs_path = os.path.abspath('/srv/data/' + filename + '.json')
        try:
            with open(self.abs_path) as savfile:
                settings = json.load(savfile)
                self.settings = settings
        except FileNotFoundError:
            temp_dict = dict()
            with open(self.abs_path, 'w') as f:
                json.dump(temp_dict, f)
            logging.info(f"***READ DATA***: Creating an empty dict at {self.abs_path} for {self.filename}.")
            with open(self.abs_path) as savfile:
                settings = json.load(savfile)
                self.settings = settings
        logging.info(f'***READ DATA***: Data type of {self.filename} found at {self.abs_path}.')

    def save_locally(self):
        with open(self.abs_path, 'w+') as json_file:
            json.dump(self.settings, json_file, indent=4)

    def save_clone(self, new_path):
        abs_path = os.path.abspath('C:\\ProgramData\\Microscope\\' + new_path + '.json')
        with open(abs_path, 'w+') as json_file:
            json.dump(self.settings, json_file, indent=4)