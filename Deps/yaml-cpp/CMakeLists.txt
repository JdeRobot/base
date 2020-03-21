find_package(yaml-cpp REQUIRED)

if (YAML_CPP_INCLUDE_DIR)
	message(STATUS "yaml-cpp found")
	list(APPEND DEPS libyaml-cpp0.5v5)
	list(APPEND DEPS_DEV libyaml-cpp-dev)
else()
	message (STATUS "${BoldYellow}yaml-cpp not found${ColourReset}")
endIF()
