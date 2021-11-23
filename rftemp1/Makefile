PART = t85
PORT = /dev/spidev0.0
BAUD = 10000
AVRFLAGS = -mmcu=attiny85 -fshort-enums -g -Isrc/
AVRCFLAGS = $(AVRFLAGS) -Os -std=c++17 -mcall-prologues -DF_CPU=1000000
AVRSFLAGS = $(AVRFLAGS) -x assembler-with-cpp

all: bin/firmware.dump

.PHONY : upload fuses clean

upload: bin/firmware.elf
	sudo avrdude -b $(BAUD) -c linuxspi -p $(PART) -P $(PORT) -U flash:w:bin/firmware.elf

upload_remote: bin/firmware.elf
	cat bin/firmware.elf | ssh pi@raspberrypi.lan "cat > /tmp/firmware.elf; avrdude -b $(BAUD) -c linuxspi -p $(PART) -P $(PORT) -U flash:w:/tmp/firmware.elf"

#fuses:
#	sudo avrdude -b $(BAUD) -c linuxspi -p $(PART) -P $(PORT) -U lfuse:w:0xF2:m

clean:
	rm -f bin/* obj/*

bin/firmware.dump: bin/firmware.elf
	avr-objdump -xd $< > $@

bin/firmware.elf: obj/firmware.o 
	avr-g++ $(AVRCFLAGS) -o$@ $^

obj/firmware.o: src/firmware.cc
	avr-g++ $(AVRCFLAGS) -c -o$@ $<
