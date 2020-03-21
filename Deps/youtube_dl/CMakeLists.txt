find_program(YOUTUBE_DL youtube-dl)
if(NOT YOUTUBE_DL)
  message(STATUS "${BoldYellow}youtube-dl not found${ColourReset}")
else()
  message(STATUS "youtube-dl found")
  list(APPEND DEPS youtube-dl)
  list(APPEND DEPS_DEV youtube-dl)
endif()



