obj-m := ptdma_bw.o
# the hosts (amy, rose) run NixOS kernel 7.2.2; build inside Coyote/shell.nix (see README.md)
KDIR ?= /nix/store/rvrswvmlrss37pnzavczhijwza4cb3q8-linux-7.2.2-dev/lib/modules/7.2.2/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
