CC=avr-gcc
AVRAR=avr-ar
AVROC=avr-objcopy
AVROD=avr-objdump
AVRUP=avrdude
AVRSIZE=avr-size
LINKER=
CPUTYPE=atmega32u4
FLASH=-R .eeprom -R .fuse -R .lock -R .signature
OPTS=-s -fno-stack-protector -fomit-frame-pointer -ffunction-sections -fdata-sections -Wl,--gc-sections -fno-unwind-tables -fno-asynchronous-unwind-tables -fno-math-errno -fno-unroll-loops -fmerge-all-constants -fno-ident -fsingle-precision-constant -ffast-math -Wl,-z,norelro -Wl,--hash-style=gnu
FLAGS=-std=c99 -mmcu=$(CPUTYPE) -Wall -Wextra -gdwarf-2 -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -DF_CPU=8000000

obj/main.o: main.c
	$(CC) $(FLAGS) -c $< -o $@ $(LINKER)

obj/main.elf: obj/main.o
	$(CC) $(FLAGS) -o $@ $< $(LINKER)
	$(AVRSIZE) --format=avr $@ --mcu=$(CPUTYPE)

bin/main.hex: obj/main.elf
	$(AVROC) -O ihex $(FLASH) $< $@

format:
	clang-format -i -style=file main.c

push: bin/main.hex
	stty -F /dev/ttyACM0 1200
	sleep 2s
	avrdude -c avr109 -p m32u4 -P /dev/ttyACM0 -b 57600 -D -U flash:w:bin/main.hex -v