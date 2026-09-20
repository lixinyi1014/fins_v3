# UART2 temporary build

This variant routes the telemetry and command UART from MCU USART6 (board silk UART1) to MCU USART1 (board silk UART2).

- Framing: 115200 baud, 8 data bits, even parity, 1 stop bit (8E1).
- Board UART2 pins: RXD, TXD, GND, 5V; use only RXD/TXD/GND with a 3.3 V USB-TTL adapter.
- Flash start: 0x08000000.
- This is a temporary hardware-observation build; propulsion power must remain disconnected during bring-up.
- The original `Firmware/FinsROV_ESKF.hex` remains the UART1/USART6 build.
