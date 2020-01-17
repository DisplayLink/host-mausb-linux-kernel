#
# Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
#
# This file is subject to the terms and conditions of the GNU General Public
# License v2. See the file COPYING in the main directory of this archive for
# more details.
#

ccflags-$(CONFIG_HOST_MAUSB_DEBUG) := -DDEBUG

obj-$(CONFIG_HOST_MAUSB) += host_mausb.o
host_mausb-y := src/hcd/hub.o
host_mausb-y += src/hcd/module_init.o
host_mausb-y += src/hcd/vhcd.o
host_mausb-y += src/hpal/data_common.o
host_mausb-y += src/hpal/data_in.o
host_mausb-y += src/hpal/data_out.o
host_mausb-y += src/hpal/hpal.o
host_mausb-y += src/hpal/isoch_in.o
host_mausb-y += src/hpal/isoch_out.o
host_mausb-y += src/hpal/mausb_events.o
host_mausb-y += src/hpal/network_callbacks.o
host_mausb-y += src/link/mausb_ip_link.o
host_mausb-y += src/utils/mausb_data_iterator.o
host_mausb-y += src/utils/mausb_mmap.o
host_mausb-y += src/utils/mausb_ring_buffer.o

ccflags-y += -I$(srctree)/$(src)/include
ccflags-y += -g
ccflags-y += -DMAUSB_WITH_LOGS
