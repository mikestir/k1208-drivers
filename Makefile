all:	net sd

clean:	net-clean sd-clean

net:
	make -C net

net-clean:
	make -C net clean

sd:
	make -C sd

sd-clean:
	make -C sd clean

.PHONY: all clean net net-clean sd sd-clean
