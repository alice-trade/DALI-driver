KERN_SRC:= /lib/modules/$(shell uname -r)/build
PWD   := $(shell pwd)
obj-m := DALI_driver.o
all:
	make -C $(KERN_SRC) M=$(PWD) modules
clean:
	make -C $(KERN_SRC) M=$(PWD) clean

util:
	gcc DALI_app.c -o dali_send
	gcc DALI_init.c -o dali_init
