LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE := edbg
LOCAL_CFLAGS := 
LOCAL_SRC_FILES := \
  dap.c \
  dbg_sysfs.c \
  edbg.c \
  target_atmel_cm0p.c \
  target_atmel_cm3.c \
  target_atmel_cm4.c \
  target_atmel_cm7.c \
  target.c
LOCAL_LDLIBS :=
LOCAL_STATIC_LIBRARIES :=
LOCAL_SHARED_LIBRARIES :=
include $(BUILD_EXECUTABLE)

