add_executable(MyEngine
  main.cpp
)

# Yep, that's it!
target_link_libraries(MyEngine
  PUBLIC
    ImGui-SFML::ImGui-SFML
)

include(Install.cmake)