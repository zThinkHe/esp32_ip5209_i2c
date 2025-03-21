set(SRCS
        "main.cpp"
        "drivers/ip5209/ip5209_driver.cpp"
)

set(INCLUDE_DIRS . "drivers/ip5209")

idf_component_register(
    SRCS ${SRCS} 
    INCLUDE_DIRS ${INCLUDE_DIRS}       
)