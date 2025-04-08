set(SRCS
        "main.cpp"
        "src/ip5209_driver.cpp"
)

set(INCLUDE_DIRS . "src")

idf_component_register(
    SRCS ${SRCS} 
    INCLUDE_DIRS ${INCLUDE_DIRS}       
)