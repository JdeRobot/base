# Copyright (C) 2007 by Cristóbal Carnero Liñán
# grendel.ccl@gmail.com
#
# This file is part of cvBlob.
#
# cvBlob is free software: you can redistribute it and/or modify
# it under the terms of the Lesser GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# cvBlob is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# Lesser GNU General Public License for more details.
#
# You should have received a copy of the Lesser GNU General Public License
# along with cvBlob.  If not, see <http://www.gnu.org/licenses/>.
#


set(cvBlob_CPP cvblob.cpp
                   cvlabel.cpp
		   cvaux.cpp
		   cvcontour.cpp
		   cvtrack.cpp
		   cvcolor.cpp
)

set(cvBlob_H cvblob.h)


set_source_files_properties(${cvBlob_CPP} 
                            PROPERTIES
                            COMPILE_FLAGS "-O3"
)

add_library(cvblob SHARED ${cvBlob_CPP} ${cvBlob_H})

target_link_libraries(cvblob ${OPENGL_LIBRARIES})

INSTALL (TARGETS cvblob DESTINATION lib/jderobot)
INSTALL (FILES ${cvblob_H} DESTINATION include/jderobot/visionlib/cvblob/)
