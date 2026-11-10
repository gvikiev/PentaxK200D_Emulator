idf_component_register(
    SRCS
        "main.c"
        "usb_device.c"
        "pentax_cam_model.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_tinyusb
        esp_timer
)

# This project implements the Mass Storage Class callbacks itself
# (tud_msc_scsi_cb and friends, in usb_device.c) to serve the Pentax
# vendor-SCSI protocol directly -- CONFIG_TINYUSB_MSC_ENABLED stays OFF
# (see sdkconfig.defaults) so esp_tinyusb's own storage-backed MSC glue
# (tinyusb_msc.c, which defines those same callback names around real
# SPI-flash/SD/blockdev storage) never gets compiled in and collides with
# ours. That Kconfig option is also what esp_tinyusb's tusb_config.h uses
# to derive CFG_TUD_MSC itself, though, so without it TinyUSB's own MSC
# class driver (src/class/msc/msc_device.c) wouldn't get built either.
#
# main/tusb_config.h works around that: it #include_next's esp_tinyusb's
# real tusb_config.h and overrides just CFG_TUD_MSC (and the endpoint
# buffer size CFG_TUD_MSC_EP_BUFSIZE, which that file never defines since
# it's normally only set when the Kconfig option is on). For #include_next
# to find esp_tinyusb's copy as "the next one", and not itself, this
# directory has to come first on the tinyusb library's own include path --
# hence the BEFORE here.
idf_build_get_property(build_components BUILD_COMPONENTS)
if(espressif__tinyusb IN_LIST build_components)
    set(pentax_tinyusb_target espressif__tinyusb)
else()
    set(pentax_tinyusb_target tinyusb)
endif()
idf_component_get_property(pentax_tusb_lib ${pentax_tinyusb_target} COMPONENT_LIB)
target_include_directories(${pentax_tusb_lib} BEFORE PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
