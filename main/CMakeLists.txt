idf_component_register(SRCS "gatts.c" "main.cpp" 
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++11)

