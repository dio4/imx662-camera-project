IMX662 CAMERA INTEGRATION PROJECT
=================================

OVERVIEW
--------
This project demonstrates successful integration of Sony IMX662 camera sensor 
with Raspberry Pi 4, achieving direct RAW capture and processing without 
OpenCV dependencies.

HARDWARE REQUIREMENTS
--------------------
- Raspberry Pi 4
- Sony IMX662 camera sensor
- Compatible CSI ribbon cable

SOFTWARE ENVIRONMENT
-------------------
- Linux Kernel: 6.11.0-26-generic
- Python: 3.x
- Required packages: numpy, cv2 (for final processing only)

KEY FEATURES
-----------
- Direct /dev/video0 access bypassing OpenCV limitations
- 10-bit RAW capture with proper RG10 format handling
- Real-time Bayer demosaicing (RG → BGR conversion)
- Multiple processing modes (grayscale, blur, edge detection, enhancement)
- Performance: ~3.22 FPS practical throughput

PROJECT STRUCTURE
-----------------
demo_imx662_direct_fixed.py      - Main demo script (5 examples)
imx662_direct_capture_final.py   - Core capture functionality
convert_raw_to_rgb.py            - RAW to RGB conversion utility
images/                          - Sample output images
documentations/                  - Technical documentation
src/                            - Kernel module (compiled)

QUICK START
----------
1. Load the kernel module: sudo insmod src/imx662.ko
2. Run demo: python3 demo_imx662_direct_fixed.py
3. Check images/ folder for results

TECHNICAL ACHIEVEMENT
--------------------
Successfully resolved "green frames" issue by:
- Bypassing OpenCV VideoCapture limitations
- Direct device read with proper frame sizing
- Correct 10-bit to 8-bit conversion
- Native Bayer pattern processing

PERFORMANCE RESULTS
------------------
- Theoretical: 18.91 FPS
- Practical: 3.22 FPS
- Frame size: 1936×1100 pixels
- Data range: 10-bit (0-1023) properly converted

FUTURE DEVELOPMENT
-----------------
See documentations/c_cpp_asm_integration_plan.md for planned C/C++ migration 
targeting 10x performance improvement (33+ FPS).

LICENSE
-------
Open source project for educational and research purposes. 