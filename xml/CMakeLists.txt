install(FILES sample_tactile_sensor.xml sample_hfield.png
        DESTINATION "${CMAKE_INSTALL_DATADIR}/xml")

if(USE_ROS2)
  install(FILES sample_tactile_sensor_ros.xml
        DESTINATION "${CMAKE_INSTALL_DATADIR}/xml")
endif()
