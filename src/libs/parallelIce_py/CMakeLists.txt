cmake_minimum_required(VERSION 2.8)

configure_module_python(parallelIce)

add_custom_target(parallelIce_py ALL)
copy_to_binary_python(parallelIce_py parallelIce)

install_python(parallelIce core)
