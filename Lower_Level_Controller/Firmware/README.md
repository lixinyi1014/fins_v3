# 可烧录固件

FinsROV_ESKF.hex：推荐烧录文件。BIN用于需要裸二进制的工具，起址0x08000000。AXF保留调试符号。

这是固定ESKF + SI控制版本，上电停止，压力校准和反馈正常后仍需ON与持续HB。默认VOFA FireWater 16通道，角度为度、深度为米。固件标识ESKF_VOFA_20260916_R2。尚未上板验证。

首次操作见 [ST-Link与VOFA步骤](../docs/BOARD_BRINGUP.md)，接线见 [V3.3端口表](../docs/PORT_MAP_V33.md)。校验值和构建记录见build-info.json。
主Keil输出目录MDK-ARM/RM_Frame_C也已放入同版AXF/HEX，可在核对ST-Link设置后直接Download，无需把HEX加进源码工程。源码修改后需先重新编译。

本次预构建来自本机 Keil 5.43a / Arm Compiler 6.24 的实际工程构建（O2，无LTO，保留NaN/无效浮点检查）。
Keil重新编译源码时，更新的是MDK-ARM/RM_Frame_C里的输出，不自动覆盖本目录。
