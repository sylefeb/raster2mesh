CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

ADD_SUBDIRECTORY(LibSL)

# ADD_DEFINITIONS(-DUSE_GLUX -DOPENGL -DOPENGL4)

SET(SHADERS
	sh_raster
	sh_clear
	sh_init
	sh_count
	sh_store
	sh_allocate
	sh_update
	sh_gentris
	sh_mesh
	sh_implicit
)
AUTO_BIND_SHADERS( ${SHADERS} )

ADD_EXECUTABLE(raster2mesh
	raster2mesh.cpp
	sh_raster.fp
	sh_raster.vp
	sh_raster.gs
	sh_raster.h
	sh_cvt.fp
	sh_mesh.fp
	sh_mesh.vp
	sh_mesh.h
	sh_clear.fp
	sh_clear.vp
	sh_clear.h
	sh_count.fp
	sh_count.vp
	sh_count.h
	sh_init.fp
	sh_init.vp
	sh_init.h
	sh_store.fp
	sh_store.vp
	sh_store.h
	sh_allocate.fp
	sh_allocate.vp
	sh_allocate.h
	sh_implicit.vp
	sh_implicit.fp
	sh_implicit.h
	sh_update.fp
	sh_update.vp
	sh_update.h
	sh_gentris.fp
	sh_gentris.vp
	sh_gentris.h
	)

TARGET_LINK_LIBRARIES(raster2mesh LibSL LibSL_gl4)

IF(WIN32)
INSTALL(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/release/raster2mesh.exe" DESTINATION ${CMAKE_SOURCE_DIR}/bin)
ELSE(WIN32)
INSTALL(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/raster2mesh" DESTINATION ${CMAKE_SOURCE_DIR}/bin)
ENDIF(WIN32)
