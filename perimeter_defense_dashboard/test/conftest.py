import os
import sys

# Allow running the tests from the source tree without a ROS install.
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'perimeter_defense_world'))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'perimeter_defense_viz'))
