obj-m := ptdma_bw.o ptdma_q.o
# amy and rose run NixOS kernel 7.2.6, clara 7.2.2; build inside Coyote/shell.nix (see README.md):
#   make KDIR=/nix/store/a6bzjg7xjlsp8z0nwjkbmq0q22nvnlqq-linux-7.2.6-dev/lib/modules/7.2.6/build   (amy, rose)
#   make KDIR=/nix/store/rvrswvmlrss37pnzavczhijwza4cb3q8-linux-7.2.2-dev/lib/modules/7.2.2/build   (clara)
KDIR ?= /nix/store/a6bzjg7xjlsp8z0nwjkbmq0q22nvnlqq-linux-7.2.6-dev/lib/modules/7.2.6/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
