cmake_minimum_required(VERSION 2.6) # Pour éviter un warning au lancement de CMake, on doit préciser la version minimale

# Nom du projet
project(IK_PF)

include_directories(
	/usr/include/OGRE/
	/usr/include/eigen3/
	/usr/local/include/
	/usr/local/include/OGRE/
	./dep/include/
	./dep/include/eigen3/
)

link_directories(
	../dep/lib/
	/usr/lib/
	/usr/local/lib/
	/usr/local/lib/OGRE/
)

SET(EXECUTABLE_OUTPUT_PATH ../bin/)


file(
    GLOB_RECURSE
    source_files
    src/*
)

set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-msse3")
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-msse3")
#set (CMAKE_CXX_FLAGS "-m32")
#set (CMAKE_C_FLAGS "-m32")
#set(CMAKE_MODULE_LINKER_FLAGS -m32)
#set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-msse2")
#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-msse2")
#set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-msse")
#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-msse")
#set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-march=")
#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-march=")
#set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-mfpmath=sse")
#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-mfpmath=sse")


# Déclaration de l'exécutable
add_executable(
    filtering
    ${source_files}
)

target_link_libraries(
    filtering
	OgreMain
	OIS
	libyaml-cpp.a
)


