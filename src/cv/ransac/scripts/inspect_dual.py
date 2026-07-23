import math

import h5py
import matplotlib.pyplot as plt
import numpy as np
from ransac.common import CameraPosition, GridConfiguration
from ransac.pipeline import DepthSegementation, HDF5Source


def main() -> None:
    filepath = input("Filename of recording (*.hdf5): ")
    try:
        file = h5py.File(filepath, "r")
    except FileNotFoundError:
        print(f'[err] There is no .hdf5 recording at path "{filepath}"')
        return
    left = HDF5Source(file, 0)
    right = HDF5Source(file, 1)

    # everything you need to set up
    conf = GridConfiguration(5000.0, 5000.0, 50.0)
    depseg = DepthSegementation(
        [(left, CameraPosition(0, 0, math.radians(20))), (right, CameraPosition(250, 200, 0))], conf
    )

    print(f"using frame number: {left.use_frame(-1)}")
    right.use_frame(left.frame_number)

    # run every frame data update (check timestamps are different)
    depseg.process()

    # everything after here is just matplotlib

    f, axs = plt.subplot_mosaic([["lmask", "rmask", "diff"], ["lgrid", "rgrid", "final"]])
    axs["lmask"].set_title("left mask")
    axs["lmask"].imshow(depseg.masks[0], cmap="gray")
    axs["lgrid"].set_title("left grid")
    axs["lgrid"].imshow(depseg.grids[0], cmap="gray")

    axs["rmask"].set_title("right mask")
    axs["rmask"].imshow(depseg.masks[1], cmap="gray")
    axs["rgrid"].set_title("right grid")
    axs["rgrid"].imshow(depseg.grids[1], cmap="gray")

    axs["diff"].set_title("difference grid")
    axs["diff"].imshow(depseg.merge_simple(np.logical_xor), cmap="gray")
    axs["final"].set_title("combined grid")
    axs["final"].imshow(depseg.merge_grids(), cmap="gray")

    f.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
