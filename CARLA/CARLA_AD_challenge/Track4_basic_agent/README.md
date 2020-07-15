# Track 4 basic rule-based agent

Track 4 初步决策、规划、控制框架实现，参考代码https://github.com/paulyehtw/Motion-Planning-on-CARLA 。

- 为了方便快速功能实现，对ObjectiveFinder传感器进行了修改。

  具体修改代码`${CARLA_ROOT}/PythonAPI/carla/scene_layout.py`文件中的`get_dynamic_objects`函数中的`get_vehicles`函数如下：

  ```python
    def get_vehicles(vehicles):
        vehicles_dict = dict()
        for vehicle in vehicles:
            v_transform = vehicle.get_transform()
            v_speed = vehicle.get_velocity()
            location_gnss = carla_map.transform_to_geolocation(v_transform.location)
            v_dict = {
                "id": vehicle.id,
                "position": [location_gnss.latitude, location_gnss.longitude, location_gnss.altitude],
                "location": [v_transform.location.x, v_transform.location.y, v_transform.location.z],
                "orientation": [v_transform.rotation.roll, v_transform.rotation.pitch, v_transform.rotation.yaw],
                "speed": [v_speed.x, v_speed.y, v_speed.z],
                "bounding_box": [[v.longitude, v.latitude, v.altitude] for v in _get_bounding_box(vehicle)]
            }
            vehicles_dict[vehicle.id] = v_dict
        return vehicles_dict
  ```

  注：目前无法通过GPS坐标系坐标计算世界坐标系坐标，故加入车辆在世界坐标系下的坐标。另加入车辆速度。

- 增加agent函数返回当前规划路径点，并直观显示在server中，方便debug。

运行代码：

`python test_scenario.py --scenario FollowLeadingVehicle_12 --reloadWorld --agent=Track4Agent.py --configFile FollowLeadingVehicle.xml` 

注：只能运行简单地图（如Town01）下的场景，否则会出现跟SceneLayout传感器一样的卡死现象。为测试规划效果，在代码中人为添加静态障碍物在固定位置，按上述指定代码运行即可，否则须修改`test_scenario.py`中生成静态障碍物对应的部分代码。
