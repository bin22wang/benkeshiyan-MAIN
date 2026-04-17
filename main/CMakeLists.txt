# 注册主程序及各功能模块源文件
idf_component_register(
    SRCS
        "app_main.c"
        "bsp_i2c.c"
        "modules/wifi_mgr.c"
        "modules/udp_comm.c"
        "modules/led_ctrl.c"
        "modules/recon_mgr.c"
        "modules/formation_algo.c"
    "modules/proto_parser.c"
    "modules/vel_solver.c"
    "modules/uart.c"
        "modules/tof_ctrl.c"
    "modules/calib_mgr.c"
    "modules/calib_store.c"
    INCLUDE_DIRS
        "."          # 使 app_main.c 能用 #include "modules/xxx.h"
        "modules"    # 使模块内部互相 #include "xxx.h" 无需路径前缀
    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        lwip
        driver
        disp_mgr
        esp32-vl53l0x-master
        AT24C02
)
