#
# (C) Copyright 2007
# Stefan Roese, DENX Software Engineering, sr@denx.de.
# (C) Copyright 2008 Freescale Semiconductor
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

NAND_SPL := y
TEXT_BASE := 0xfff00000
PAD_TO := 0xfff04000

include $(TOPDIR)/config.mk

LDSCRIPT= $(TOPDIR)/nand_spl/board/$(BOARDDIR)/u-boot.lds
LDFLAGS	= -Bstatic -T $(LDSCRIPT) -Ttext $(TEXT_BASE) $(PLATFORM_LDFLAGS)
AFLAGS	+= -DCONFIG_NAND_SPL
CFLAGS	+= -DCONFIG_NAND_SPL

SOBJS	= start.o ticks.o
COBJS	= nand_boot_fsl_elbc.o $(BOARD).o sdram.o ns16550.o nand_init.o \
	  time.o cache.o

SRCS	:= $(addprefix $(obj),$(SOBJS:.o=.S) $(COBJS:.o=.c))
OBJS	:= $(addprefix $(obj),$(SOBJS) $(COBJS))
__OBJS	:= $(SOBJS) $(COBJS)
LNDIR	:= $(OBJTREE)/nand_spl/board/$(BOARDDIR)

nandobj	:= $(OBJTREE)/nand_spl/

ALL	= $(nandobj)u-boot-spl $(nandobj)u-boot-spl.bin $(nandobj)u-boot-spl-16k.bin

all:	$(obj).depend $(ALL)

$(nandobj)u-boot-spl-16k.bin: $(nandobj)u-boot-spl
	$(OBJCOPY) ${OBJCFLAGS} --pad-to=$(PAD_TO) -O binary $< $@

$(nandobj)u-boot-spl.bin:	$(nandobj)u-boot-spl
	$(OBJCOPY) ${OBJCFLAGS} -O binary $< $@

$(nandobj)u-boot-spl:	$(OBJS)
	cd $(LNDIR) && $(LD) $(LDFLAGS) $(__OBJS) \
		-Map $(nandobj)u-boot-spl.map \
		-o $(nandobj)u-boot-spl

# create symbolic links for common files

$(obj)start.S:
	ln -sf $(SRCTREE)/cpu/mpc83xx/start.S $(obj)start.S

$(obj)nand_boot_fsl_elbc.c:
	ln -sf $(SRCTREE)/nand_spl/nand_boot_fsl_elbc.c \
	       $(obj)nand_boot_fsl_elbc.c

$(obj)sdram.c:
	ln -sf $(SRCTREE)/board/$(BOARDDIR)/sdram.c $(obj)sdram.c

$(obj)$(BOARD).c:
	ln -sf $(SRCTREE)/board/$(BOARDDIR)/$(BOARD).c $(obj)$(BOARD).c

$(obj)ns16550.c:
	ln -sf $(SRCTREE)/drivers/serial/ns16550.c $(obj)ns16550.c

$(obj)nand_init.c:
	ln -sf $(SRCTREE)/cpu/mpc83xx/nand_init.c $(obj)nand_init.c

$(obj)cache.c:
	ln -sf $(SRCTREE)/lib_ppc/cache.c $(obj)cache.c

$(obj)time.c:
	ln -sf $(SRCTREE)/lib_ppc/time.c $(obj)time.c

$(obj)ticks.S:
	ln -sf $(SRCTREE)/lib_ppc/ticks.S $(obj)ticks.S

#########################################################################

$(obj)%.o:	$(obj)%.S
	$(CC) $(AFLAGS) -c -o $@ $<

$(obj)%.o:	$(obj)%.c
	$(CC) $(CFLAGS) -c -o $@ $<

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
