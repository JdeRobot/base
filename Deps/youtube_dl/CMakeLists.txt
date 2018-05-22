find_program(YOUTUBE_DL youtube-dl)
if(NOT YOUTUBE_DL)
  message("youtube-dl not found")
else()
  message("youtube-dl found at " ${YOUTUBE_DL})
  list(APPEND DEPS youtube-dl)
  list(APPEND DEPS_DEV youtube-dl)
endif()



