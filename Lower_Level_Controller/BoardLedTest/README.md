# RoboMaster C 板裸板亮灯测试

这个目录是独立测试程序，不修改 `Lower_Level_Controller` 的整机控制程序。

程序只做三件事：

1. 开启 STM32F407 的 GPIOH 时钟；
2. 将 PH10、PH11、PH12 配置为普通推挽输出；
3. 红、绿、蓝依次点亮，再全部熄灭，循环运行。

## 引脚

| 引脚 | C 板标号 |
|---|---|
| PH10 | LED_B，蓝灯 |
| PH11 | LED_G，绿灯 |
| PH12 | LED_R，红灯 |

该程序不初始化外部晶振、USB、UART、I2C、SPI、PWM 扩展板、压力计或 FreeRTOS，因此裸 C 板即可测试。

## 构建

在 PowerShell 中进入本目录，执行：

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\build.ps1
```

输出文件在 `build`：

- `BoardLedTest.hex`：推荐给 STM32CubeProgrammer；
- `BoardLedTest.bin`：二进制格式，写入地址为 `0x08000000`；
- `BoardLedTest.axf`：保留调试符号；
- `BoardLedTest.map`：构建检查信息。

## 用 USB DFU 烧录

1. 拔掉 USB，BOOT0 设为 1，BOOT1 设为 0；
2. 插 USB，按一次 RST；
3. STM32CubeProgrammer 选择 `USB`，连接 `USB1`；
4. 选择 `build/BoardLedTest.hex`；
5. 勾选写入后校验，点击 Start Programming；
6. 断开 USB；
7. 恢复 BOOT0=0、BOOT1=0；
8. 再插 USB 或按 RST。

预期结果是红、绿、蓝依次变化，之后短暂熄灭，再循环。此程序没有蜂鸣器代码，声音没有变化属于预期。

## 如果仍然不亮

- 确认 BOOT0 已恢复为 0；
- 确认 CubeProgrammer 报告 verify 成功；
- 确认 USB 线在重新插拔后给板子供电；
- 用万用表确认板上 3.3 V 电源存在；
- 不要把整机固件和本测试固件混淆：本测试固件只验证 MCU 和板载 LED。
