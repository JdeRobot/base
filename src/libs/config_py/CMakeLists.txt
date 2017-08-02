cmake_minimum_required(VERSION 2.8)

configure_module_python(jderobotconfig)

add_custom_target(jderobotconfig_py ALL)
copy_to_binary_python(jderobotconfig_py jderobotconfig)

install_python(jderobotconfig core)
