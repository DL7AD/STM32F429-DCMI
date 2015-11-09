# STM32F429-DCMI
DCMI testing with STM32F429-DISCOVERY and OV9655 camera

This program is made for a STM32F429 discovery board AND OV9655 camera. The demo program samples a 320x240px image from a OV9655 camera into the SRAM and encodes it into JPEG in real time. It then transmits the image by serial to the computer where it can be displayed. The image is encoded into JPEG in real time because the STM32F429 doesn't provide enough memory to hold a raw image (320x240px) in it's SRAM.

The image is encoded to JPEG while the microcontroller receives it by DCMI. The image is read linewise. Every 16 lines, an interrupt is generated, so the algorithm can encode 16x16px blocks at the same time.

Sources:
- DCMI example http://mikrocontroller.bplaced.net/wordpress/?page_id=1115
- JPEGant http://sourceforge.net/projects/jpegant.berlios/
- To make this demo work ChibiOS is needed https://github.com/ChibiOS/ChibiOS
