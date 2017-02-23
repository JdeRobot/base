cmake_minimum_required(VERSION 2.8)

configure_module_python(jderobotTypes)

add_custom_target(jderobotTypes_py ALL)
copy_to_binary_python(jderobotTypes_py jderobotTypes)

install_python(jderobotTypes core)
