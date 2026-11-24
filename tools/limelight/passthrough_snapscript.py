# Beacon pass-through SnapScript for the Limelight 3A.
#
# Paste this into a Python (SnapScript) pipeline in the Limelight web UI
# (http://limelight.local:5801). It returns every camera frame UNCHANGED: no contour, no drawing,
# no text. Beacon needs the clean image, because anything drawn on it (crosshair, boxes) would
# look like field edges to the vision code.
#
# The Limelight calls runPipeline(image, llrobot) once per frame (Limelight docs,
# "Python SnapScript Pipelines"). It must return three things:
#   1. a contour for the crosshair/targeting readings (we return an empty one),
#   2. the image to stream (we return the input image itself),
#   3. llpython, a list of numbers sent to the robot (we send eight zeros).
import numpy as np


def runPipeline(image, llrobot):
    empty_contour = np.array([[]])
    llpython = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    return empty_contour, image, llpython
