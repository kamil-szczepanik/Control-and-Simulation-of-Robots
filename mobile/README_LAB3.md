<node pkg="amcl" type="amcl" name="amcl">
	<remap from="scan" to="scan_raw"/>
</node>

<!--<node ..... static_transform_publisher ...
	-->

1. rviz -> particlecloud

2. services:
	global_localization

3. teleop

4. zobaczyc...

relacje pomiedzy ukladami w trakcie ruchu:
	map	
	odom
	base_link

parametry w wiki
testy w budynku i na korytarzu
wnioski do sprawka

wykorzystywac service global_localization