$ErrorActionPreference = 'Stop'
$tool = 'D:\keil\keil5.43\ARM\ARMCLANG\bin'
$out = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $out | Out-Null
& "$tool\armclang.exe" --target=arm-arm-none-eabi -mcpu=cortex-m4 -mthumb -Oz -ffreestanding -fno-builtin -c (Join-Path $PSScriptRoot 'main.c') -o (Join-Path $out 'main.o')
& "$tool\armclang.exe" --target=arm-arm-none-eabi -mcpu=cortex-m4 -mthumb -c (Join-Path $PSScriptRoot 'startup_stm32f407xx.s') -o (Join-Path $out 'startup.o')
& "$tool\armlink.exe" --cpu=Cortex-M4 -d --diag_suppress=L6306W --entry=Reset_Handler --first=g_pfnVectors --ro-base=0x08000000 --rw-base=0x20000000 (Join-Path $out 'startup.o') (Join-Path $out 'main.o') --output (Join-Path $out 'BoardLedTest.axf')
& "$tool\fromelf.exe" --i32combined (Join-Path $out 'BoardLedTest.axf') --output (Join-Path $out 'BoardLedTest.hex')
& "$tool\fromelf.exe" --bin (Join-Path $out 'BoardLedTest.axf') --output (Join-Path $out 'BoardLedTest.bin')
& "$tool\fromelf.exe" --text -c (Join-Path $out 'BoardLedTest.axf') --output (Join-Path $out 'BoardLedTest.map')
Write-Host "Built: $out"
