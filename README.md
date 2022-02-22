# iam-construct
tools for constructing and running the iam interface

## Installation
To install everything, run: `sudo ./iam-construct-install.sh -a`

## Running
To run everything, run: `./iam-construct-run.sh <ip>`
Where `<ip>` is the IP address of the Franka's control computer that you are connected to. You can also add the `-v` argument after the IP address to run the point cloud to voxel publisher. For help, run `./iam-construct-run.sh -h`