cmake_minimum_required(VERSION 2.8)

configure_module_python(connector)

add_custom_target(connector_py ALL)
copy_to_binary_python(connector_py connector)

install_python(connector connector-python)
