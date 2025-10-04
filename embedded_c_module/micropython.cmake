add_library(usermod_neo_m8 INTERFACE)

target_sources(usermod_neo_m8 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/neo_m8.c
)

target_include_directories(usermod_neo_m8 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_neo_m8)