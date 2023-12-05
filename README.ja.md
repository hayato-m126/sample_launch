# sample_launch

ros2のlaunchの仕様を確認するためのパッケージ

## 参考

<https://github.com/ros2/launch/blob/rolling/launch/launch/actions/group_action.py#L49-L65>

## 結論

groupのデフォルトの条件はscoped=True、forwarding=Trueであり、groupで囲っただけではargは暗黙のうち渡る。
forwarding=Falseにすると伝搬は防げるがlaunchにargを記述して明示的に渡す必要がある。

## 疑問

autowareはforwarding属性指定していないようなので、引数がforwardingされてしまう。
名前が同じ引数があり、それをlaunchコマンドで指定した場合、想定しない挙動になる可能性がある。
autowareはpythonとxmlと複数のlaunchで構成されているが同じ名前の引数がないことはどうやって確認しているのか？

## インストール

```shell
mkdir $HOME/sample_ws/src -p
cd $HOME/sample_ws/src
git clone https://github.com/hayato-m126/sample_launch.git
cd $HOME/sample_ws
colcon build --symlink-install --catkin-skip-building-tests --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
```

## 動作確認準備

以降のコマンド実行前にインストール先のsetup.bashをsourceする

```shell
cd $HOME/sample_ws
source install/setup.bash
```

## argが暗黙のうちに転送される

<https://github.com/autowarefoundation/autoware.universe/issues/2695#issuecomment-1397876624>

```shell
❯ ros2 launch sample_launch parent.launch.xml 
[INFO] [launch.user]: parent_foo
[INFO] [launch.user]: parent_bar
```

## 同じ名前のargがあると最初に読み込まれ方で上書きされる

<https://github.com/ros2/launch/issues/620>

### groupを付けずにincludeした場合

```shell
❯ ros2 launch sample_launch outside.launch.xml 
[INFO] [launch.user]: include1 has 1
[INFO] [launch.user]: include2 has 1
```

### groupを付けてincludeした場合

```shell
❯ ros2 launch sample_launch outside_group.xml 
[INFO] [launch.user]: include1 has 1
[INFO] [launch.user]: include2 has 2
```

### groupを付けてincludeしてlaunchの引数でtest_argを指定した場合

一見groupをつければ解決しているように見えるが、launchの引数で渡してしまうと同じ名前の引数なので両方同じ値で上書きされてしまう。

```shell
❯ ros2 launch sample_launch outside_group.xml test_arg:=3
[INFO] [launch.user]: include1 has 3
[INFO] [launch.user]: include2 has 3
```

### groupを付けてforwarding=falseにする

forwardingしないので、-sでargumentを出力しても何も出てこない。
当然引数を指定して実行しても、それぞれ違うものとして扱われる

```shell
❯ ros2 launch sample_launch outside_forwarding_false.xml -s
Arguments (pass arguments as '<name>:=<value>'):

  No arguments.

❯ ros2 launch sample_launch outside_forwarding_false.xml 
[INFO] [launch.user]: include1 has 1
[INFO] [launch.user]: include2 has 2

❯ ros2 launch sample_launch outside_forwarding_false.xml test_arg:=3
[INFO] [launch.user]: include1 has 1
[INFO] [launch.user]: include2 has 2
```
