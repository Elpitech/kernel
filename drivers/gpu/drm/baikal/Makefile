# SPDX-License-Identifier: GPL-2.0
baikal_vdu_drm-y +=	baikal_vdu_connector.o \
		baikal_vdu_crtc.o \
		baikal_vdu_drv.o \
		baikal_vdu_encoder.o \
		baikal_vdu_gem.o \
		baikal_vdu_plane.o

baikal_vdu_drm-$(CONFIG_DEBUG_FS) += baikal_vdu_debugfs.o

obj-$(CONFIG_DRM_BAIKAL_VDU) += baikal_vdu_drm.o
obj-$(CONFIG_DRM_BAIKAL_HDMI) += baikal-hdmi.o
