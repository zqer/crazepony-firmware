copy /Y bin\cflie.elf .\
arm-none-eabi-objcopy cflie.elf -O ihex cflie.hex
arm-none-eabi-objcopy cflie.elf -O binary --pad-to 0 cflie.bin
