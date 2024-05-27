from setuptools import setup

setup(
    name="OrsaySystem",
    version="0.1",
    author="Yves Auad",
    description="Set of tools to run a VG Microscope in Nionswift",
    url="https://github.com/yvesauad/swift_lumiere",
    packages=['nionswift_plugin.aux_files', 'nionswift_plugin.diaf', 'nionswift_plugin.EELS_spec',
              'nionswift_plugin.IVG', 'nionswift_plugin.IVG.camera', 'nionswift_plugin.IVG.QuantumPY',
              'nionswift_plugin.IVG.scan', 'nionswift_plugin.IVG.tp3', 'nionswift_plugin.IVG.virtual_instruments',
              'nionswift_plugin.lenses', 'nionswift_plugin.OptSpec', 'nionswift_plugin.stage'],
    python_requires='>=3.8.5',
    data_files=[('Lib/site-packages/nionswift_plugin/aux_files/config', [
        'nionswift_plugin/aux_files/config/global_settings.json',
        'nionswift_plugin/aux_files/config/camera_settings.json',
        'nionswift_plugin/aux_files/config/diafs_settings.json',
        'nionswift_plugin/aux_files/config/eels_settings.json',
        'nionswift_plugin/aux_files/config/lenses_settings.json',
        'nionswift_plugin/aux_files/config/Orsay_cameras_list.json',
        'nionswift_plugin/aux_files/config/stage_settings.json',
        'nionswift_plugin/aux_files/read_data.py'

    ]), ('Lib/site-packages/nionswift_plugin/aux_files/DLLs', [
        'nionswift_plugin/aux_files/DLLs/Cameras.dll',
        'nionswift_plugin/aux_files/DLLs/atmcd64d.dll',
        'nionswift_plugin/aux_files/DLLs/Scan.dll',
        'nionswift_plugin/aux_files/DLLs/udk3-1.4-x86_64.dll',
        'nionswift_plugin/aux_files/DLLs/udk3mod-1.4-winusb-x86_64.dll',
        'nionswift_plugin/aux_files/DLLs/udk3-1.5.1-x86_64.dll',
        'nionswift_plugin/aux_files/DLLs/udk3mod-1.5.1-winusb-x86_64.dll',
        'nionswift_plugin/aux_files/DLLs/libudk3-1.5.1.so',
        'nionswift_plugin/aux_files/DLLs/libudk3mod-1.5.1-libusb.so',
        'nionswift_plugin/aux_files/DLLs/Connection.dll',
        'nionswift_plugin/aux_files/DLLs/Connection.dll.config',
        'nionswift_plugin/aux_files/DLLs/STEMSerial.dll',
        'nionswift_plugin/aux_files/DLLs/STEMSerialLib.dll',
        'nionswift_plugin/aux_files/DLLs/Stepper.dll',
        'nionswift_plugin/aux_files/DLLs/delib64.dll',
        'nionswift_plugin/aux_files/DLLs/SpectroCL.dll',
        'nionswift_plugin/aux_files/DLLs/AttoClient.dll',
        'nionswift_plugin/aux_files/DLLs/Newtonsoft.Json.dll',
    ]
    )],
    zip_safe = False
)
