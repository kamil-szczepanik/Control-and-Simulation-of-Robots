Czyszczenie sarych wiadomości:
rosrun velma_common reset_shm_comm.py

Odpalanie systemu:
roslaunch manipulation doors.launch

Odpalenie gazebo i rviza:
roslaunch manipulation sim.launch

Odpalenie publikowania pozycji klamki:
roslaunch rcprg_gazebo_utils gazebo_publish_ros_tf_object.launch link_name:="cabinet_door_fragile::left_handle" frame_id:="cabinet_handle"

Odpalenie pick_and_place
rosrun manipulation open_door.py

=======================================================
Inicjalizacja robota:
rosrun velma_task_cs_ros_interface initialize_robot.py
------------------------------------------------
Ruch głową i korpusem:
rosrun manipulation test_head.py
------------------------------------------------
Odpalenie octomapy online:
roslaunch velma_common octomap_server.launch