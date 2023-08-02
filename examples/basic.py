from xyztank.model import *
import numpy as np

tank = Tank(
    name="small_aquarium",
    dimensions=(
        200e-3,  # OX [m]
        300e-3,  # OY [m]
        200e-3   # OZ [m]
    ),
    position=(10e-3, 7e-3, -200e-3)  # xyz
)

plan = MeasurementPlan(
    name="Example",
    tank=tank,
    is_vertical=False,
    min_position=(120e-3, 80e-3, -120e-3),  # xyz
    max_position=(125e-3, 82e-3, -110e-3),  # xyz
    grid_precision=(1e-3, 1e-3, 1e-3),
    grid=None

)

hydrophone = Hydrophone(
    name="hydrophone1",
    dimensions=(
        100e-3,  # OX [m]
        100e-3,  # OY [m]
        30e-3   # OZ [m]
    ),
    position=(15e-3, 10e-3, -180e-3),  # xyz
    safety_margin=20e-3
)

ultrasound_transducer = UltrasoundTransducer(
    name="usg1",
    dimensions=(
        100e-3,  # OX [m]
        100e-3,  # OY [m]
        30e-3   # OZ [m]
    ),
    position=(300e-3, 200e-3, -10e-3)  # xyz

)
