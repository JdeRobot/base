cmake_minimum_required(VERSION 2.8)

configure_module_python(jderobotComm)

add_custom_target(jderobotcomm_py ALL)
copy_to_binary_python(jderobotcomm_py jderobotComm)

install_python(jderobotComm core)