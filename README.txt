How to replicate the project
1. Acquire and set up a Kinect V2.
2. Scan an object into PLY format files with the Kinect for Windows SDK, Kinect Fusion application.
3. Install miniconda/anaconda for simpler environment handling.
4. Set up CUDA and dowload the latest cudatoolkit from NVIDIA.
5. For the IPC algorithms, install the open3d library with supported dependencies.
6. For the machine learning algorithm, install Python=3.6, PyTorch=1.4, CUDA=10.1, Open3D 0.8.0.

To test the full IPC reconstruction algorithm
7.1. Open either "ptpoint_ipc_full.py" or "ptplane_ipc_full.py".
7.2. Change the "num_pcds" to the number of available pcds, and set the path for "source" and "target" to the pcd directory.
7.3. Run the Python file in the terminal.

To test the full DIPs reconstruction algorithm
7.1. Open the "3dmatch_demo.py".
7.2. Change the "num_pcds" to the number of available pcds, and set the path for "pcd1" and "pcd2" to the pcd directory.
7.3. Run the Python file in the terminal.