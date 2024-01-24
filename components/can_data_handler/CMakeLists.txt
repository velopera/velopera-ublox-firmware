idf_component_register(SRCS "can_data_handler.cpp" "can_velopera.cpp"
                    INCLUDE_DIRS "include" "../../common")
target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++11)
#/home/kguzel/proj/secopera/mini-iot/repos/mini-iot-can-bus-integration/twai_network_listen_only/common
