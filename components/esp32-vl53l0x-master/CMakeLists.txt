# 自动查找 core 和 platform 目录下的所有 .c 源文件
file(GLOB_RECURSE srcs "core/*.c" "platform/*.c")

# 注册组件，并指定头文件所在的目录
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "core/inc" 
                                    "platform/inc"
                       REQUIRES driver)