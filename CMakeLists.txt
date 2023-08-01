if(ESP_PLATFORM)
    set(SOURCES I2Cdev_esp32.cpp)

    idf_component_register(
        SRCS ${SOURCES}
        INCLUDE_DIRS . 
        REQUIRES driver
    )

else(ESP_PLATFORM)
    add_library(I2Cdev I2Cdev.cpp I2Cdev.h)

    target_link_libraries(I2Cdev 
        i2c
    )

    target_include_directories(I2Cdev PUBLIC
        .
    )
endif(ESP_PLATFORM)
