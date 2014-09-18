rock_library(rosa_localization
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(rosa_localization_bin Main.cpp
    DEPS rosa_localization)

