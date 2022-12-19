import ctypes
import numpy as np
import matplotlib.pyplot as plt
import importlib.util
import sys
from time import sleep
from math import log2
from picosdk.ps5000a import ps5000a as ps
from picosdk.functions import assert_pico_ok, mV2adc, adc2mV
from picosdk.errors import PicoSDKCtypesError, PicoError
# from config import config
from logging_ import get_logger
from dict import channel, resolution

# TODO Consider using some @exception_handler decorator to handle exceptions.


class Oscilloscope:
    """
    A class responsible for managing the oscilloscope.
    """

    def __init__(self, config_path: str):
        """
        Initializes the class, imports measurement configuration and connects with the device.
        :param config_path (string) : full path to configuration file
        """
        # This attribute is assigned a value later.
        self.log = get_logger(type(self).__name__)
        self.config = self.import_config(config_path)  # Config is now dynamically imported from a specified file.
        self.chandle = ctypes.c_int16()
        self.status = dict()
        self.maxADC = ctypes.c_int16()

        self.open_connection()
        self.disable_channel(['all'])
        self.find_max_adc_val()

    def find_max_adc_val(self):
        """
        Outputs the maximum ADC count value to maxADC class field. The output value depends on the currently selected
        resolution.
        """
        self.status["maximumValue"] = ps.ps5000aMaximumValue(self.chandle, ctypes.byref(self.maxADC))
        assert_pico_ok(self.status["maximumValue"])

    def open_connection(self):
        """
        Manages connecting with the oscilloscope.
        """
        self.status["openunit"] = ps.ps5000aOpenUnit(ctypes.byref(self.chandle), None, self.config.resolution)
        try:
            assert_pico_ok(self.status["openunit"])
        except PicoSDKCtypesError:
            powerstatus = self.status["openunit"]
            # TODO Check for other possible power supply errors
            # PICO_USB3_0_DEVICE_NON_USB3_0_PORT
            # Should the user be able to choose the power source?
            if powerstatus == 286:
                self.log.debug("Device connected to USB 2.0 port, oscilloscope expects USB 3.0.")
                self.status["changePowerSource"] = ps.ps5000aChangePowerSource(self.chandle, powerstatus)
            # PICO_POWER_SUPPLY_NOT_CONNECTED
            # It's recommended to conduct measurements with the oscilloscope connected to the power source.
            elif powerstatus == 282:
                self.log.warning("Power supply unit not connected." +
                                 " Only A and B channels and no generator will be available.")
                self.status["changePowerSource"] = ps.ps5000aChangePowerSource(self.chandle, powerstatus)
            elif powerstatus == 3:
                self.log.exception("Oscilloscope is not connected!")
                raise
            else:
                self.log.exception("Encountered unexpected picosdk exception.")
                raise
            try:
                assert_pico_ok(self.status["changePowerSource"])
            except PicoError:
                self.log.exception("Error changing power source (or related settings).")
                raise
            else:
                self.log.info("Connection with the oscilloscope established.")
        else:
            self.log.info("Connection with the oscilloscope established.")

    def close_connection(self):
        """
        Manages disconnecting from the device.
        """
        self.status["closeunit"] = ps.ps5000aCloseUnit(self.chandle)
        try:
            assert_pico_ok(self.status["closeunit"])
        except PicoError:
            self.log.exception("Eror closing connection with the oscilloscope.")
            raise
        else:
            self.log.info("Connection with the oscilloscope closed.")

    def set_channel(self):
        """
        Sets the channel based on the data provided in config file. Sets data buffer and verifies sampling
         parameters.
        """
        self.status["setChannel"] = ps.ps5000aSetChannel(self.chandle, self.config.channel, 1,
                                                         self.config.coupling_type, self.config.range, 0)
        try:
            assert_pico_ok(self.status["setChannel"])
        except PicoError:
            self.log.exception(f"Erorr setting channel:")
            raise
        else:
            self.log.info("Oscilloscope channel set with config parameters.")

        self.n_samples = int(1000 * self.config.measurement_time * self.config.sampling_frequency)
        self.verify_timeinterval = ctypes.c_float()  # ns
        self.verify_n_samples = ctypes.c_int32()
        self.status["getTimebase2"] = ps.ps5000aGetTimebase2(self.chandle,
                                                             self.find_timebase(self.config.sampling_frequency),
                                                             self.n_samples, ctypes.byref(self.verify_timeinterval),
                                                             ctypes.byref(self.verify_n_samples), 0)
        self.log.info(f"Desired sampling frequency: {self.config.sampling_frequency} Msa/s")
        self.log.info(f"Verified sampling frequency: {1 / (self.verify_timeinterval.value / 1000)} MSa/s")
        # Test
        # print(f"Verified frequency: {1 / (self.verify_timeinterval.value / 1000)} MHz")
        # print(f"Desired frequency: {os.sampling_frequency} MHz")
        # print(f"Verified samples: {self.verify_n_samples.value}")  # What's that value exactly?

        # Do we want our data_buffer to be global? Maybe just put it in runMeasurement method?
        # Buffer has to be much longer than the expected received signal!
        self.data_buffer = (ctypes.c_int16 * self.n_samples)()
        self.status["setDataBuffer"] = ps.ps5000aSetDataBuffer(self.chandle, self.config.channel,
                                                               ctypes.byref(self.data_buffer), self.n_samples, 0, 0)
        try:
            assert_pico_ok(self.status["setDataBuffer"])
        except PicoError:
            self.log.exception(f"Error setting data buffer.")
            raise
        else:
            self.log.info("Data buffer is set.")

    def set_meas_trigger(self):
        """
        Sets the detection of an event which will trigger the measurement based on the data provided in config file.
        """
        self.status["trigger"] = ps.ps5000aSetSimpleTrigger(
            self.chandle, 1, self.config.trigger_source,
            int(mV2adc(self.config.trigger_threshold, self.config.range, self.maxADC)), 2,
            int(self.config.delay / (self.verify_timeinterval.value / 1000000)), 0
        )
        try:
            assert_pico_ok(self.status["trigger"])
        except PicoError:
            self.log.exception("Error setting measurement trigger.")
            raise
        else:
            self.log.info("Mesurement trigger set with config parameters.")

    def run_measurement(self):
        """
        Sets and starts the measurement process. The device will wait for the previously set trigger
         and collect the data to its memory.
        """
        self.status["runBlock"] = ps.ps5000aRunBlock(self.chandle, 0, self.n_samples,
                                                     self.find_timebase(self.config.sampling_frequency), None, 0, None,
                                                     None)
        try:
            assert_pico_ok(self.status["runBlock"])
        except PicoError:
            self.log.exception("Error starting measurment.")
            raise
        else:
            self.log.info("Measurement started. Oscilloscope is waiting for trigger.")

    # Instead of executing this function one might consider modifying it and using as a callback function,
    # which is executed when the data is ready. Morea in ps5000aBlockCallbackExample.py
    def get_data(self):
        """
        Retrieves the measurement data from the oscilloscope's memory.
        :return: samples (array): Measurement data scaled in mV.
        """
        is_ready = ctypes.c_int16(0)
        check = ctypes.c_int16(0)

        while is_ready.value == check.value:
            self.status["isReady"] = ps.ps5000aIsReady(self.chandle, ctypes.byref(is_ready))

        self.log.info("Measurement data ready to be acquired.")
        overflow = ctypes.c_int16()
        c_samples = ctypes.c_int32(self.n_samples)
        self.status["getValues"] = ps.ps5000aGetValues(self.chandle, 0, ctypes.byref(c_samples), 0, 0, 0,
                                                       ctypes.byref(overflow))
        try:
            assert_pico_ok(self.status["getValues"])
        except PicoError:
            self.log.exception("Exception acquiring data to the computer.")
            raise
        else:
            self.log.info("Measurement data acquired to the computer.")
            self.samples = adc2mV(self.data_buffer, self.config.range, self.maxADC)
            return self.samples

    def plot_data(self):
        """
        Simple matplotlib plot of the measurement data for the verification purpouses.
        """
        time = np.linspace(0, (self.n_samples - 1) * 1 / (self.config.sampling_frequency / 1000), self.n_samples)
        plt.plot(time, self.samples[:])
        plt.xlabel('Time [ns]')
        plt.ylabel('Voltage [mV]')
        plt.show()

        # Test:
        # print(f"Desired samples: {self.n_samples} ")
        # print(f"Output Samples: {len(samples)}")
        # print(f"Desired measurement time: {os.measurement_time} ms")
        # print(f"Desired impulse length: {os.impulse_length} ms")

    # Generator methods seem to work.
    # However, after starting and stopping the generator (only setting is fine),
    # there is still some constant noisy voltage left on generator output,
    # lower than the generated one. I have no idea where it's from, for now,
    # I'd reccommend setting adequate trigger threshold values, close to triggering
    # signal's peak.

    # UPDATE: Try just setting signal's amplitude to 0.
    def set_generator(self):
        """
        Sets the generator based on the data provided in config file.
        """
        # enums describing generator's settings missing in picosdk. Need to use numerical values.
        # ctypes.c_uint32(-1) - PS5000A_SHOT_SWEEP_TRIGGER_CONTINUOUS_RUN - not available as enum.
        self.status["setGenerator"] = ps.ps5000aSetSigGenBuiltInV2(self.chandle, self.config.offset_voltage,
                                                                   self.config.Vpp,
                                                                   ctypes.c_int32(self.config.wave_type),
                                                                   self.config.signal_frequency * 1000,
                                                                   self.config.signal_frequency * 1000,
                                                                   0, 1, ctypes.c_int32(0), 0,
                                                                   ctypes.c_uint32(-1), 0,
                                                                   ctypes.c_int32(2), ctypes.c_int32(4), 0)
        try:
            assert_pico_ok(self.status["setGenerator"])
        except PicoError:
            self.log.exception("Error setting the generator.")
            raise
        else:
            self.log.info("Generator set with config parameters.")

    def start_generator(self):
        """
        Enables the generator's output.
        """
        self.status["startGenerator"] = ps.ps5000aSigGenSoftwareControl(self.chandle, 1)
        try:
            assert_pico_ok(self.status["startGenerator"])
        except PicoError:
            self.log.exception("Error starting the generator")
            raise
        else:
            self.log.info("Generator started.")

    def stop_generator(self):
        """
        Disables the generator's output."
        """
        self.status["stopGenerator"] = ps.ps5000aSigGenSoftwareControl(self.chandle, 0)
        try:
            assert_pico_ok(self.status["stopGenerator"])
        except PicoError:
            self.log.exception("Error stopping the generator")
            raise
        else:
            self.log.info("Generator stopped.")

    def generate_impulse(self):
        """
        Generates an impulse of length (time) specified in config file."
        """
        self.start_generator()
        sleep(self.config.impulse_length / 1000)
        self.stop_generator()

    # noinspection PyMethodMayBeStatic
    def _return_timebase_formula(self, res: int):
        """
        Returns formula for calculating the timebase, depending on the resolution chosen in config.
        Based on picoscope5000a programming guide.
        :param res (int): chosen oscilloscope's resolution
        :return: formula (fun): function used for calculating the timebase.
        """
        def _14bitFormula(sampling_frequency: float):
            if sampling_frequency == 125.0:
                return 3
            else:
                return 125 / sampling_frequency + 2

        def _12bitFormula(sampling_frequency: float):
            if sampling_frequency in [125, 250, 500]:
                return log2(500 / sampling_frequency) + 1
            else:
                return 62.5 / sampling_frequency + 3

        def _8bitFormula(sampling_frequency: float):
            if sampling_frequency in [250, 500, 1000]:
                return log2(1000 / sampling_frequency)
            else:
                return 125 / sampling_frequency + 2

        formulas = {resolution["8BIT"]: _8bitFormula,
                    resolution["12BIT"]: _12bitFormula,
                    resolution["14BIT"]: _14bitFormula
                    }

        # formulas = {resolution["8BIT"]: lambda sf: 3 if sf == 125.0 else 125/sf+2,
        #             resolution["12BIT"]: lambda sf: log2(500/sf)+1 if sf in [125, 250, 500] else 62.5/sf+3,
        #             resolution["14BIT"]: lambda sf: log2(1000/sf) if sf in [250, 500, 1000] else 125/sf+2
        #             }

        return formulas[res]

    def find_timebase(self, sampling_frequency: float) -> int:
        """
        Returns integer timebase parameter that will allow to set desired sampling frequency (or one closest to it)
        on the oscilloscope.
        :param sampling_frequency: chosen oscilloscope's sampling frequency
        :return: n (int) : timebase parameter.
        """
        formula = self._return_timebase_formula(self.config.resolution)
        return int(formula(sampling_frequency))

    def disable_channel(self, channels: list[str]):
        """
        Disables chosen oscilloscope's channels. (At the beginning all channels are enabled)
        :param channels: List of channels to be disabled. ['all'] means all channels.
        """
        if channels[0] == 'all':
            channels = ['A', 'B', 'C', 'D']
        for ch in channels:
            try:
                self.status["setChannel"] = ps.ps5000aSetChannel(self.chandle, channel[ch], 0,
                                                                 self.config.coupling_type, self.config.range, 0)
                assert_pico_ok(self.status["setChannel"])
            except PicoError:
                self.log.exception(f"Exception disabling oscilloscope channel {ch}.")
                raise
            else:
                self.log.info(f"Disabled channel {ch}.")

    def import_config(self, path: str):
        """
        Imports config from specified file.
        :param path: Full path to the configuration file.
        :return config (Settings) : Imported config dataclass.
        """
        try:
            spec = importlib.util.spec_from_file_location("conf", path)
            conf = importlib.util.module_from_spec(spec)
            sys.modules["conf"] = conf
            spec.loader.exec_module(conf)
        except:
            self.log.exception("Exception importing config from file. Check if specified file exists.")
            raise
        else:
            self.log.info("Successfully imported config from specified file.")
            return conf.config
