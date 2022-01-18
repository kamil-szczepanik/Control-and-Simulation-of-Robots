Czyszczenie sarych wiadomości:
rosrun velma_common reset_shm_comm.py

Odpalanie systemu:
roslaunch manipulation doors.launch

Odpalenie gazebo i rviza:
roslaunch manipulation sim.launch

Odpalenie publikowania pozycji klamki:
roslaunch rcprg_gazebo_utils gazebo_publish_ros_tf_object.launch link_name:="cabinet_door_fragile::left_handle" frame_id:="cabinet_handle"

Inicjalizacja robota:
rosrun velma_task_cs_ros_interface initialize_robot.py

Odpalenie open_door
rosrun manipulation open_door.py

=======================================================
Inicjalizacja robota:
rosrun velma_task_cs_ros_interface initialize_robot.py
-------------------------------------------------------


### Zbieranie i zapisanie octomapy:

Odpalenie octomapy online:
`roslaunch velma_common octomap_server.launch`

Ruch głową i korpusem:
`rosrun manipulation test_head.py`

Zapisanie octomapy
`rosrun octomap_server octomap_saver  -f name_of_map.bt`

