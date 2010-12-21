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

# Phytec 3250 board with or without S1L
#

#
# 64 or 128 MB SDRAM @ 0x80000000
#
# Linux-Kernel is @ 0x80008000, entry 0x80008000
# params @ 0x80000100
# optionally with a ramdisk at 0x80300000
#
# we load ourself to 0x00000000 or 0x83FC0000
#
# download area is 0x80f00000
#

TEXT_BASE = 0x83F80000

