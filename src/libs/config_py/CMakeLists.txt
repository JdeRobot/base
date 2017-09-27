cmake_minimum_required(VERSION 2.8)

configure_module_python(config)

add_custom_target(config_py ALL)
copy_to_binary_python(config_py config)

install_python(config core)
