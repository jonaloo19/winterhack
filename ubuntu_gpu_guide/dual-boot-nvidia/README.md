# Ubuntu 22.04 with NVIDIA GPU on a Dual-Boot Machine

This guide provides a **step-by-step walkthrough** for configuring **Ubuntu 22.04.5 LTS with native NVIDIA GPU acceleration** on a **dual-boot system alongside Windows 11**.

Unlike WSL, Ubuntu in a dual-boot setup has **direct, native access to the NVIDIA GPU**, enabling full support for:
- OpenGL / Vulkan
- CUDA
- Gazebo / RViz
- Machine learning workloads
- Real-time graphics and robotics simulation

---

## Machine Configuration Used

> The steps below are validated on the following hardware and software configuration.

- **Laptop:** Dell XPS 15 9500 
- **Linux OS:** Ubuntu 22.04.5 LTS  
- **GPU:** NVIDIA GeForce RTX 1650 Ti Mobile  
- **NVIDIA Driver:** 580.95.05

> ⚠️ Other NVIDIA GPUs should work similarly. Secure Boot, hybrid graphics, and firmware settings may affect installation.

---

## Table of Contents

- [Step 1. Prepare Windows for Dual-Boot](#step-1-prepare-windows-for-dual-boot)
- [Step 2. Install NVIDIA Driver on Ubuntu](#step-2-install-nvidia-driver-on-ubuntu)
- [Step 3 Verify Native GPU Acceleration](#step-3-verify-native-gpu-acceleration)
- [Step 4 Hybrid Graphics & PRIME (Laptops)](#step-4-verify-hybrid-graphics--opengl-renderer)
- [Step 5. Recommended Ubuntu Workflow](#step-5-launch-gazebo-with-nvidia-gpu)

---

## Step 1. Prepare Windows for Dual-Boot
Setting up a dual-boot system involves several **Windows-side preparation steps** that are  available online.

To keep this guide focused on **Ubuntu 22.04 and NVIDIA GPU configuration**, you should first follow an existing dual-boot installation guide to prepare your system.

A recommended reference is:
- **LinuxTechi – Dual Boot Ubuntu 22.04 and Windows 11**  
  https://www.linuxtechi.com/dual-boot-ubuntu-22-04-and-windows-11/


## Step 2. Install NVIDIA Driver on Ubuntu

### 2.1 Identify GPU

```bash
lspci | grep -i nvidia
```

---

### 2.2 Install Recommended Driver

```bash
sudo apt update
sudo ubuntu-drivers devices
sudo ubuntu-drivers autoinstall
sudo reboot
```

---

## Step 3. Verify Native GPU Acceleration

### 3.1 Native Linux Rendering Stack

```text
Linux OpenGL / Vulkan application
        ↓
NVIDIA proprietary OpenGL / Vulkan driver
        ↓
Kernel NVIDIA module
        ↓
Physical NVIDIA GPU
```
---

### 3.2 Check NVIDIA Driver

```bash
nvidia-smi
```

---

## Step 4. Verify Hybrid Graphics & OpenGL Renderer

Most modern laptops use **hybrid graphics (Optimus)**, where an **Intel GPU** is used for the desktop and light workloads, while the **NVIDIA GPU** is activated only for GPU-intensive applications. This configuration provides the best balance between **performance and power efficiency**.

In this setup, it is **expected and correct** for the desktop session to run on the Intel GPU, while selected applications are explicitly offloaded to the NVIDIA GPU.

---

### 4.1 First, ensure the required tools are installed:
Run:
```bash
sudo apt update
sudo apt install mesa-utils -y
sudo apt install nvidia-prime -y
```

---

### 4.2 Check the current PRIME mode:
Run:
```bash
prime-select query
```
Expected output:
```text
on-demand
```
This confirms that the system is configured for on-demand GPU offloading, which is the recommended mode for hybrid laptops.

---

### 4.3 Verify Default OpenGL Renderer
Run:
```bash
glxinfo | grep -E "OpenGL vendor|OpenGL renderer|OpenGL version"
```
Expected output:
```text
OpenGL vendor string: Intel
OpenGL renderer string: Mesa Intel(R) UHD ...
```
---
### 4.4 Verify NVIDIA GPU Offloading
```bash
prime-run glxinfo | grep -E "OpenGL vendor|OpenGL renderer|OpenGL version"
```
Expected output:
```text
OpenGL vendor string: NVIDIA Corporation
OpenGL renderer string: NVIDIA GeForce GTX ...
```

This confirms that NVIDIA PRIME offloading is working correctly, and that applications launched with `prime-run` are rendered using the native NVIDIA proprietary driver rather than Mesa.

---

## Step 5. Launch Gazebo with NVIDIA GPU

### 5.1 For a single app, use `prime-run` for NVIDIA GPU offloading; desktop GPU stays on Intel
Run:
```bash
prime-run ros2 launch robot_gazebo worlds.launch.py world_name:=maze_world nav:=false
```
For persistent NVIDIA GPU offloading, add the environment parameters below to `.bashrc`:

```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
```
---

### 5.2 Entire system set to Nvidia GPU
Run:
```bash
sudo prime-select nvidia
```
Verify:
```bash
prime-select query
```
Expected output:
```text
nvidia
```


Note: On some laptops (including mine), the above environment variables are still required for `watch nvidia-smi` to show activity even when PRIME is set to `nvidia`.

---


## Notes: Dual-Boot vs WSL

| Feature | Dual-Boot Ubuntu | WSL2 |
|------|----------------|------|
| GPU access | Native | D3D12 passthrough |
| CUDA | Full | Supported |
| Gazebo | Best | Good |
| Kernel access | Full | Limited |
| Real-time | Yes | No |

---
