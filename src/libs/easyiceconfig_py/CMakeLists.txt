cmake_minimum_required(VERSION 2.8)

configure_module_python(easyiceconfig)

add_custom_target(easyiceconfig_py ALL)
copy_to_binary_python(easyiceconfig_py easyiceconfig)

install_python(easyiceconfig core)
