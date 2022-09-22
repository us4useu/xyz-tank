from picosdk.ps5000a import ps5000a as ps

channel = {"A": ps.PS5000A_CHANNEL["PS5000A_CHANNEL_A"],
           "B": ps.PS5000A_CHANNEL["PS5000A_CHANNEL_B"],
           "C": ps.PS5000A_CHANNEL["PS5000A_CHANNEL_C"],
           "D": ps.PS5000A_CHANNEL["PS5000A_CHANNEL_D"],
           "EXT": ps.PS5000A_CHANNEL["PS5000A_EXTERNAL"]
}

resolution = {
    "8BIT": ps.PS5000A_DEVICE_RESOLUTION["PS5000A_DR_8BIT"],
    "12BIT": ps.PS5000A_DEVICE_RESOLUTION["PS5000A_DR_12BIT"],
    "14BIT": ps.PS5000A_DEVICE_RESOLUTION["PS5000A_DR_14BIT"]
}

coupling = {
    "DC": ps.PS5000A_COUPLING["PS5000A_DC"],
    "AC": ps.PS5000A_COUPLING["PS5000A_AC"]
}

range = {
    "10mV": ps.PS5000A_RANGE["PS5000A_10MV"],
    "20mV": ps.PS5000A_RANGE["PS5000A_20MV"],
    "50mV": ps.PS5000A_RANGE["PS5000A_50MV"],
    "100mV": ps.PS5000A_RANGE["PS5000A_100MV"],
    "200mV": ps.PS5000A_RANGE["PS5000A_200MV"],
    "500mV": ps.PS5000A_RANGE["PS5000A_500MV"],
    "1V": ps.PS5000A_RANGE["PS5000A_1V"],
    "2V": ps.PS5000A_RANGE["PS5000A_2V"],
    "5V": ps.PS5000A_RANGE["PS5000A_5V"],
    "10V": ps.PS5000A_RANGE["PS5000A_10V"],
    "20V": ps.PS5000A_RANGE["PS5000A_20V"],
    "50V": ps.PS5000A_RANGE["PS5000A_50V"],
    "max": ps.PS5000A_RANGE["PS5000A_MAX_RANGES"]
}

wave_type = {
    "SINE": 0,
    "SQUARE": 1,
    "TRIANGLE": 2,
    "RAMP_UP": 3,
    "RAMP_DOWN": 4,
    "SINC": 5,
    "GAUSSIAN": 6,
    "HALF_SINE": 7,
    "DC": 8  # Doesn't work for (yet) unknown reasons.
}