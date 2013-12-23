waiterbot
=========

#### Pre-requisite ####

```
sudo pip install -U yujin_tools
```

#### Installation ####

```
yujin_init_workspace -j10 --track=hydro ~/waiterbot https://raw.github.com/yujinrobot/kobuki-x/hydro-devel/waiterbot/waiterbot.rosinstall
cd ~/waiterbot
source /opt/ros/hydro/setup.bash
rosdep update
rosdep install --from-paths src /opt/ros/hydro --ignore-src --rosdistro hydro -y
yujin_init_build . -u /opt/ros/hydro
yujin_make
```

#### Prepare hardware ####

IR sensors ring requires a bit of extra work. Check their [launch file](https://github.com/yujinrobot/kobuki-x/blob/hydro-devel/waiterbot/launch/includes/_ir_sensors.launch) for instructions.
