#--------------------------------------------------------------------------
# example project
#--------------------------------------------------------------------------

FILE(GLOB example_headers code/*.h)
FILE(GLOB example_sources code/*.cc)

SET(files_example ${example_headers} ${example_sources})
SOURCE_GROUP("example" FILES ${files_example})
message(${CMAKE_SOURCE_DIR})

ADD_EXECUTABLE(example ${files_example})
TARGET_LINK_LIBRARIES(example core render)
SET_TARGET_PROPERTIES(example PROPERTIES CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

ADD_DEPENDENCIES(example core render)
