import os
import sys
import warnings

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
sys.path.insert(0, HERE)
# numpy 2.2 on Apple Accelerate prints spurious matmul warnings on large finite arrays
warnings.filterwarnings("ignore", message=".*encountered in matmul")
