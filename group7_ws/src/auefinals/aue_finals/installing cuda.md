### This guide is only appplicable for Ubuntu 20.04 with GTX 1660Ti.
#### Tested on Apr 18,2020; Pranava Swaroopa
OS: Ubuntu
Version: 20.04
Graphics Card: GTX 1660 Ti
CUDA Version:  NVIDIA-SMI 450.172.01   Driver Version: 450.172.01   CUDA Version: 11.0
1. Check for the version of CUDA using nvidia-smi
2. Download and Install CUDA-Toolkit using the following commands (Installing CUDA-Toolkit 11.0.3)
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
    sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
    wget https://developer.download.nvidia.com/compute/cuda/11.0.3/local_installers/cuda-repo-ubuntu2004-11-0-local_11.0.3-450.51.06-1_amd64.deb
    sudo dpkg -i cuda-repo-ubuntu2004-11-0-local_11.0.3-450.51.06-1_amd64.deb
    sudo apt-key add /var/cuda-repo-ubuntu2004-11-0-local/7fa2af80.pub
    sudo apt-get update
    sudo apt-get -y install cuda
3. Reboot
4. Check nvidia-smi: NVIDIA-SMI 510.47.03    Driver Version: 510.47.03    CUDA Version: 11.6
5. export PATH=/usr/local/cuda-11.6/bin${PATH:+:${PATH}}
6. Open CMakeLists.txt foudnd in darknet_ros direcctory. 
  Remove everything with written under the below flag and only keep/add the compute capability of GTX 1660 Ti, which is 7.5
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_75,code=sm_75
7. In the carkin_ws
    catkin_make -DCMAKE_BUILD_TYPE=Release