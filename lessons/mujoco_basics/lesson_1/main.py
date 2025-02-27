import os

import numpy as np
import mujoco
import mujoco.viewer


"""
    In this file we will learn how to locate a mujoco xml file
    and get its file path using the os module from the standard library.

    We will also learn how to load the xml file and simulate the system.
"""


def main(argv=None):
    filepath = os.path.join(os.path.dirname(__file__), 'cart_pole.xml') # Cross-platform file path.

    # Load the xml file.
    model = mujoco.MjModel.from_xml_path(filepath)
    print('Model loaded successfully.')

    # Update the timestep.
    model.opt.timestep = 0.002

    data = mujoco.MjData(model)
    print('Data object loaded successfully.')

    data.qpos = np.array([-1.0, -np.pi/4])
    data.qvel = np.array([0.0, 0.0])
    data.ctrl = np.array([0.0])
    mujoco.mj_forward(model, data) # Connect the data to the model.
    print('Connected the data to the model')

    # Visualize and simulate the system.
    # Send the model and data to the viewer i.e. update the context for the viewer.
    with mujoco.viewer.launch_passive(model, data) as viewer: # Context manager.
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == '__main__':
    main()
