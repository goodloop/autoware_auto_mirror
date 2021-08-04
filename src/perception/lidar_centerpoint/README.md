# CenterPoint TensorRT

This is a 3D object detection implementation of CenterPoint supporting TensorRT inference.

The models are trained with [OpenPCDet](https://github.com/tier4/OpenPCDet)

## Install Dependencies
### CUDA 11.1
Follow the [official instructions](https://developer.nvidia.com/cuda-11.1.0-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=2004&target_type=debnetwork) or run the following commands.

```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub
echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /" | sudo tee /etc/apt/sources.list.d/nvidia-ml.list
sudo apt update
sudo apt install cuda-11-1
echo 'export PATH="/usr/local/cuda/bin:$PATH"' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"' >> ~/.bashrc
```

### TensorRT 7
Follow the instructions in: https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html#installing-debian

### LibTorch
* Go to https://pytorch.org/
* Download "Stable", "Linux", "LibTorch", "C++/Java", "CUDA 11.1" (x11 ABI)
* Extract and copy to `/usr/local/libtorch`
* install libgomp1: `sudo apt install libgomp1`
* replace libgomp library file in libtorch:
* `sudo mv /usr/local/libtorch/lib/libgomp-75eea7e8.so.1 /usr/local/libtorch/lib/libgomp-75eea7e8.so.1.backup`
* `sudo ln -s /usr/lib/x86_64-linux-gnu/libgomp.so.1 /usr/local/libtorch/lib/libgomp-75eea7e8.so.1`
* echo 'export LD_LIBRARY_PATH="/usr/local/libtorch/lib:$LD_LIBRARY_PATH"' >> ~/.bashrc

## Parameters

### Input Topics

| Name             | Type        | Description                          |
| ---------------- | ----------- | ------------------------------------ |
| input/pointcloud | PointCloud2 | Point Clouds (x, y, z and intensity) |

### Output Topics

| Name                           | Type                          | Description            |
| ------------------------------ | ----------------------------- | ---------------------- |
| output/objects                 | DynamicObjectWithFeatureArray | 3D Bounding Box        |
| debug/pointcloud_densification | PointCloud2                   | multi-frame pointcloud |

### ROS Parameters

| Name                      | Type   | Description                                                 | Default |
| ------------------------- | ------ | ----------------------------------------------------------- | ------- |
| score_threshold           | float  | detected objects with score less than threshold are ignored | `0.4`   |
| densification_base_frame  | string | the base frame id to fuse multi-frame pointcloud            | `map`   |
| densification_past_frames | int    | the number of past frames to fuse with the current frame    | `1`     |
| use_cuda_preprocessor     | bool   | transform pointcloud to voxel-format on GPU                 | `false` |
| use_vfe_trt               | bool   | use TensorRT VoxelFeatureEncoder                            | `false` |
| use_head_trt              | bool   | use TensorRT DetectionHead                                  | `true`  |
| trt_precision             | string | TensorRT inference precision: `fp32` or `fp16`              | `fp16`  |
| vfe_onnx_path             | string | path to VoxelFeatureEncoder ONNX file                       |         |
| vfe_engine_path           | string | path to VoxelFeatureEncoder TensorRT Engine file            |         |
| vfe_pt_path               | string | path to VoxelFeatureEncoder TorchScript file                |         |
| head_onnx_path            | string | path to DetectionHead ONNX file                             |         |
| head_engine_path          | string | path to DetectionHead TensorRT Engine file                  |         |
| head_pt_path              | string | path to DetectionHead TorchScript file                      |         |



## For Developers

If you have an error like `'GOMP_4.5' not found`,  replace the OpenMP library in libtorch.

```
sudo apt install libgomp1 -y
rm /path/to/libtorch/lib/libgomp-75eea7e8.so.1
ln -s /usr/lib/x86_64-linux-gnu/libgomp.so.1 /path/to/libtorch/lib/libgomp-75eea7e8.so.1
```



## Reference

Yin, Tianwei, Xingyi Zhou, and Philipp Krähenbühl. "Center-based 3d object detection and tracking." arXiv preprint arXiv:2006.11275 (2020).



## Reference Repositories

- https://github.com/tianweiy/CenterPoint
- https://github.com/open-mmlab/OpenPCDet
- https://github.com/poodarchu/Det3D
- https://github.com/xingyizhou/CenterNet
- https://github.com/lzccccc/SMOKE
- https://github.com/yukkysaito/autoware_perception
- https://github.com/pytorch/pytorch

