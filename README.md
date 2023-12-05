# sample_launch

Package for checking launch specification of ros2

## Reference

<https://github.com/ros2/launch/blob/rolling/launch/launch/actions/group_action.py#L49-L65>

## Conclusion

The default conditions of group are scoped=True and forwarding=True, and arg is implicitly passed across just by enclosing it in group.
If forwarding=False, propagation can be prevented, but it is necessary to write the arg in launch and pass it explicitly.

## Question

Since autoware does not seem to specify forwarding attribute, the argument is forwarded.
If there is an argument with the same name and it is specified in the launch command, it may cause unexpected behavior.
How do you check that there are no arguments with the same name when autoware consists of python, xml and multiple launches?

## install

```shell
mkdir $HOME/sample_ws/src -p
cd $HOME/sample_ws/src
git clone https://github.com/hayato-m126/sample_launch.git
cd $HOME/sample_ws
colcon build --symlink-install --catkin-skip-building-tests --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
````

## Preparation for operational verification

Source the setup.bash file of the installation directory before executing the following commands.

```shell
cd $HOME/sample_ws
source install/setup.bash
```

## arg is implicitly transferred

<https://github.com/autowarefoundation/autoware.universe/issues/2695#issuecomment-1397876624>

```shell
❯ ros2 launch sample_launch parent.launch.xml 
[INFO] [launch.user]: parent_foo
[INFO] [launch.user]: parent_bar
```

## If there are args with the same name, the first one loaded will overwrite the others

<https://github.com/ros2/launch/issues/620>

### include without group

```shell
❯ ros2 launch sample_launch outside.launch.xml 
[INFO] [launch.user]: include1 has 1
[INFO] [launch.user]: include2 has 1
```

### include with group

```shell
❯ ros2 launch sample_launch outside_group.xml 
[INFO] [launch.user]: include1 has 1
[INFO] [launch.user]: include2 has 2
```

### If you include with group and specify test_arg as an argument to launch

At first glance, adding group seems to solve the problem, but if you pass it in the launch argument, it will overwrite both with the same value because they are arguments with the same name.

```shell
❯ ros2 launch sample_launch outside_group.xml test_arg:=3
[INFO] [launch.user]: include1 has 3
[INFO] [launch.user]: include2 has 3
```

### add group and forwarding=false

Since forwarding is not done, nothing appears when outputting an argument with -s option.
Of course, if you run the program with arguments, they are treated as different arguments.

```shell
❯ ros2 launch sample_launch outside_forwarding_false.xml -s
Arguments (pass arguments as '<name>:=<value>'): No arguments.

  No arguments.

❯ ros2 launch sample_launch outside_forwarding_false.xml 
[INFO] [launch.user]: include1 has 1
[INFO] [launch.user]: include2 has 2

❯ ros2 launch sample_launch outside_forwarding_false.xml test_arg:=3
[INFO] [launch.user]: include1 has 1
[INFO] [launch.user]: include2 has 2
```
