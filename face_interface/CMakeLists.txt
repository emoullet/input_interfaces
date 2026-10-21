cmake_minimum_required(VERSION 3.8)
project(face_interface)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_face_launch
    test/test_face_launch.py
  )
  ament_add_pytest_test(test_face_pipeline
    test/test_face_pipeline.py
    TIMEOUT 90
  )
endif()

ament_package()
