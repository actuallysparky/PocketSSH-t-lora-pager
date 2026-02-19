option(TPAGER_DIAG "Build T-Pager diagnostic firmware instead of full PocketSSH UI" OFF)
option(TPAGER_TARGET "Enable T-Pager target code paths" OFF)

if(TPAGER_DIAG)
    # Diagnostic firmware is intentionally minimal so hardware bring-up loops are fast.
    set(SOURCES
        "tpager_diag.cpp"
        "tpager_xl9555.cpp"
        "tpager_tca8418.cpp"
    )
else()
    set(SOURCES
        "deck_base.cpp"
        "battery_measurement.cpp"
        "c3_keyboard.cpp"
        "ssh_terminal.cpp"
        "lvgl_pepboy_img/pepboy_0.c"
        "lvgl_pepboy_img/pepboy_1.c"
        "lvgl_pepboy_img/pepboy_2.c"
        "lvgl_pepboy_img/pepboy_3.c"
        "lvgl_pepboy_img/pepboy_4.c"
        "lvgl_pepboy_img/pepboy_5.c"
        "lvgl_pepboy_img/pepboy_6.c"
        "lvgl_pepboy_img/pepboy_7.c"
    )
endif()

# List of include directories
set(INCLUDE_DIRS
    "."
    "include"
)

if(TPAGER_DIAG)
    set(REQUIRED_COMPONENTS
        driver
        esp_timer
        nvs_flash
    )
else()
    set(REQUIRED_COMPONENTS
        lvgl
        nvs_flash
        esp_lcd_touch_gt911
        fatfs
        sdmmc
        esp_wifi
        esp_netif
        esp_event
        esp_timer
    )
endif()

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES ${REQUIRED_COMPONENTS}
)

# Add version number as compile definition
target_compile_definitions(${COMPONENT_LIB} PRIVATE POCKETSSH_VERSION="${PROJECT_VER}")

if(TPAGER_TARGET)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TPAGER_TARGET=1)
endif()

if(TPAGER_DIAG)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TPAGER_DIAG=1 TPAGER_TARGET=1)
endif()
