# HTC Vive Cosmos Elite on Linux - Setup Guide

This document details the process of getting an HTC Vive Cosmos Elite VR headset working on Linux (Pop!_OS 24.04) with SteamVR.

## Hardware Configuration

- **Headset**: HTC Vive Cosmos Elite with **External Tracking Faceplate** (lighthouse-based tracking)
- **Base Stations**: SteamVR 1.0 Base Stations (channels B and C for optical sync)
- **GPU**: NVIDIA GeForce RTX 4060
- **OS**: Pop!_OS (Ubuntu-based) with kernel 6.17.9
- **Display Protocol**: DisplayPort (DP-3)

> **Important**: The Cosmos Elite with external tracking faceplate uses lighthouse base stations for positional tracking, unlike the regular Cosmos which uses inside-out camera tracking. This is a critical distinction for driver support.

## Repository

The patched Monado source code is available at:
**https://github.com/aaddrick/monado-cosmos-elite**

## The Problem

SteamVR on Linux has limited support for the Cosmos Elite because:
1. SteamVR's lighthouse driver requires a `viveVR` peer driver that only exists on Windows
2. The headset reports as "VIVE Cosmos External Tracking" which isn't recognized by open-source drivers
3. NVIDIA's Vulkan direct mode has issues with display acquisition on X11

## What Works

- **Device Detection**: libsurvive and Monado successfully detect the Cosmos Elite
- **Lighthouse Tracking**: Base stations are detected and tracking data is received
- **Device Identification**: Monado correctly identifies the headset as "HTC Vive Cosmos Elite (libsurvive)"
- **Display Detection**: The headset display is recognized (2880x1700 @ 90Hz)

## What Doesn't Work (Yet)

- **NVIDIA Direct Mode on X11**: `vkAcquireXlibDisplayEXT` fails with `VK_ERROR_UNKNOWN`
- **SteamVR Integration**: Cannot bypass the viveVR peer driver requirement

## Software Components

### 1. libsurvive
Open-source lighthouse tracking library that handles communication with base stations and tracking.

```bash
# Clone and build
git clone https://github.com/cntools/libsurvive.git
cd libsurvive
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 2. Monado (Patched)
Open-source OpenXR runtime, patched to support Cosmos Elite.

```bash
# Clone
git clone https://gitlab.freedesktop.org/monado/monado.git monado-src
cd monado-src

# Or use the pre-patched version:
git clone https://github.com/aaddrick/monado-cosmos-elite.git monado-src
cd monado-src

# Build dependencies
sudo apt install build-essential cmake libvulkan-dev libx11-dev libxxf86vm-dev \
    libxrandr-dev libxcb-randr0-dev libx11-xcb-dev libudev-dev libsystemd-dev \
    libeigen3-dev libcjson-dev glslang-tools libhidapi-dev libusb-1.0-0-dev

# Configure and build
mkdir build && cd build
cmake .. -DXRT_BUILD_DRIVER_SURVIVE=ON \
         -DXRT_HAVE_OPENGL_GLX=OFF \
         -DXRT_FEATURE_WINDOW_PEEK=OFF
make -j$(nproc)
sudo make install
```

## Patches Applied to Monado

### 1. Add VIVE_VARIANT_COSMOS_ELITE enum
**File**: `src/xrt/auxiliary/vive/vive_common.h`

```c
enum VIVE_VARIANT
{
    VIVE_UNKNOWN = 0,
    VIVE_VARIANT_VIVE,
    VIVE_VARIANT_PRO,
    VIVE_VARIANT_INDEX,
    VIVE_VARIANT_PRO2,
    VIVE_VARIANT_BEYOND,
    VIVE_VARIANT_COSMOS_ELITE,  // Added
};
```

### 2. Add model number detection
**File**: `src/xrt/auxiliary/vive/vive_common.c`

```c
} else if (strcmp(model_number, "VIVE Cosmos External Tracking") == 0) {
    variant = VIVE_VARIANT_COSMOS_ELITE;
    U_LOG_D("Found HTC Vive Cosmos Elite HMD");
}
```

### 3. Add config parsing for Cosmos Elite
**File**: `src/xrt/auxiliary/vive/vive_config.c`

Added display parameters:
- `lens_horizontal_separation = 0.063`
- `h_meters = 0.068`
- `eye_to_screen_distance = 0.024`

Added config parsing case with lighthouse support (no camera tracking for external tracking variant).

### 4. Fix buffer overflow in _get_lighthouse()
**File**: `src/xrt/auxiliary/vive/vive_config.c`

**The Bug**: The original code allocated the sensor array based on `map_size`, but indexed it using channel numbers from `channelMap` which could be larger than the array size.

**The Fix**:
```c
// First pass: find max channel value
uint32_t max_channel = 0;
cJSON_ArrayForEach(item, json_map) {
    int map_item = 0;
    u_json_get_int(item, &map_item);
    map[i++] = (uint32_t)map_item;
    if ((uint32_t)map_item > max_channel) {
        max_channel = (uint32_t)map_item;
    }
}

// Allocate based on max channel value + 1
struct lh_sensor *s = U_TYPED_ARRAY_CALLOC(struct lh_sensor, max_channel + 1);
```

### 5. Add device name handling
**File**: `src/xrt/drivers/vive/vive_device.c`

```c
case VIVE_VARIANT_COSMOS_ELITE:
    snprintf(d->base.str, XRT_DEVICE_NAME_LEN, "HTC Vive Cosmos Elite (vive)");
    break;
```

### 6. Add survive driver support
**File**: `src/xrt/drivers/survive/survive_driver.c`

```c
case VIVE_VARIANT_COSMOS_ELITE:
    snprintf(survive->base.str, XRT_DEVICE_NAME_LEN, "HTC Vive Cosmos Elite (libsurvive)");
    break;
```

### 7. Add to NVIDIA direct mode allowlist
**File**: `src/xrt/compositor/main/comp_settings.h`

```c
"HTC Corporation VIVE Cosmos", // HTC Vive Cosmos Elite
```

## System Configuration

### udev Rules
Create `/etc/udev/rules.d/60-htc-vive-cosmos.rules`:

```
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0bb4", ATTRS{idProduct}=="0313", MODE="0660", TAG+="uaccess"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0bb4", ATTRS{idProduct}=="0313", MODE="0660", TAG+="uaccess"
```

Then reload:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### SteamVR Launch Options
Add to Steam launch options for SteamVR:
```
QT_QPA_PLATFORM=xcb %command%
```

### OpenXR Runtime Configuration
```bash
mkdir -p ~/.config/openxr/1
echo '{"file_format_version":"1.0.0","runtime":{"library_path":"/usr/local/lib/libopenxr_monado.so"}}' > ~/.config/openxr/1/active_runtime.json
```

## Testing

### Test libsurvive
```bash
survive-cli
# Should show: "T20 is treated as HMD device" and lighthouse detection
```

### Test Monado
```bash
monado-cli probe
# Should show: "HTC Vive Cosmos Elite (libsurvive)" with 2 views
```

## Current Blockers

### NVIDIA Vulkan Direct Mode on X11
The NVIDIA driver's `vkAcquireXlibDisplayEXT` returns `VK_ERROR_UNKNOWN` when trying to acquire the headset display. This is a known limitation of NVIDIA's Linux Vulkan implementation.

**Potential Workarounds**:
1. Run from a TTY (Ctrl+Alt+F3) without X11
2. Use Wayland instead of X11
3. Use WiVRn for wireless VR streaming
4. Wait for NVIDIA driver improvements

### SteamVR viveVR Driver
SteamVR's lighthouse driver hard-codes a requirement for the `viveVR` peer driver which only exists on Windows. This cannot be bypassed without SteamVR source code modifications.

## Files Modified

| File | Changes |
|------|---------|
| `src/xrt/auxiliary/vive/vive_common.h` | Added VIVE_VARIANT_COSMOS_ELITE enum |
| `src/xrt/auxiliary/vive/vive_common.c` | Added model number detection |
| `src/xrt/auxiliary/vive/vive_config.c` | Added config parsing, display params, buffer overflow fix |
| `src/xrt/drivers/vive/vive_device.c` | Added device name and IMU handling |
| `src/xrt/drivers/survive/survive_driver.c` | Added device name for libsurvive |
| `src/xrt/compositor/main/comp_settings.h` | Added to NVIDIA allowlist |

## References

- [Monado OpenXR Runtime](https://monado.freedesktop.org/)
- [libsurvive](https://github.com/cntools/libsurvive)
- [SteamVR on Linux](https://github.com/ValveSoftware/SteamVR-for-Linux)
- [Hachi-VR Project](https://github.com/mBornuo/Hachi-VR) - Related Cosmos reverse engineering

## License

Monado patches follow the original Monado license (Boost Software License 1.0).

---

*Document generated with assistance from Claude Code*
