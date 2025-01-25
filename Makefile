CC=avr-gcc
AVRAR=avr-ar
AVROC=avr-objcopy
AVROD=avr-objdump
AVRUP=avrdude
AVRSIZE=avr-size
LINKER=
CPUTYPE=atmega328p
OBJ=obj/backend.o obj/rf95.o
FLASH=-R .eeprom -R .fuse -R .lock -R .signature
OPTS=-s -fno-stack-protector -fomit-frame-pointer -ffunction-sections -fdata-sections -Wl,--gc-sections -fno-unwind-tables -fno-asynchronous-unwind-tables -fno-math-errno -fno-unroll-loops -fmerge-all-constants -fno-ident -fsingle-precision-constant -ffast-math -Wl,-z,norelro -Wl,--hash-style=gnu
FLAGS=-std=c99 -mmcu=$(CPUTYPE) -Wall -Wextra -O1 -funsigned-char -fpack-struct -fshort-enums -DF_CPU=16000000 -Werror

sender: bin/sender.hex
	rm -f obj/*

reciever: bin/reciever.hex
	rm -f obj/*

bin/reciever.hex: FLAGS+=-DRECIEVER
bin/sender.hex:   FLAGS+=-DSENDER

bin/%.hex: obj/%.elf
	$(AVROC) -O ihex $(FLASH) $< $@

obj/%.elf: obj/%.o $(OBJ)
	$(CC) $(FLAGS) -o $@ $< $(OBJ) $(LINKER)
	$(AVRSIZE) --format=avr $@ --mcu=$(CPUTYPE)

obj/%.o: %.c
	$(CC) $(FLAGS) -c $< -o $@ $(LINKER)

#format:
#	indent -linux *.c
#	rm -f *~


push: clean bin/main.hex
	avrdude -P /dev/ttyACM0 -c arduino -p m328p -U flash:w:bin/main.hex -v

pull:
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -D -U eeprom:r:bin/eeprom.hex -v

clean:
	rm -f obj/* bin/*
