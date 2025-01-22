CC=avr-gcc
AVRAR=avr-ar
AVROC=avr-objcopy
AVROD=avr-objdump
AVRUP=avrdude
AVRSIZE=avr-size
LINKER=
CPUTYPE=atmega328p
FLASH=-R .eeprom -R .fuse -R .lock -R .signature
OPTS=-s -fno-stack-protector -fomit-frame-pointer -ffunction-sections -fdata-sections -Wl,--gc-sections -fno-unwind-tables -fno-asynchronous-unwind-tables -fno-math-errno -fno-unroll-loops -fmerge-all-constants -fno-ident -fsingle-precision-constant -ffast-math -Wl,-z,norelro -Wl,--hash-style=gnu
FLAGS=-std=c99 -mmcu=$(CPUTYPE) -Wall -Wextra -O1 -funsigned-char -fpack-struct -fshort-enums -DF_CPU=16000000 -Werror

all: clean bin/main.hex

obj/%.o: %.c
	$(CC) $(FLAGS) -c $< -o $@ $(LINKER)

obj/%.elf: obj/%.o
	$(CC) $(FLAGS) -o $@ $< $(LINKER)
	$(AVRSIZE) --format=avr $@ --mcu=$(CPUTYPE)

bin/main.hex: obj/main.elf
	$(AVROC) -O ihex $(FLASH) obj/main.elf $@

format:
	indent -linux *.c
	rm -f *~
test: obj/rf95.o obj/led.o

push: clean bin/main.hex
	avrdude -P /dev/ttyACM0 -c arduino -p m328p -U flash:w:bin/main.hex -v

pull:
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -D -U eeprom:r:bin/eeprom.hex -v

clean:
	rm -f obj/*
