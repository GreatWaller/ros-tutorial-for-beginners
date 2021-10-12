# 动手学ROS（12）：多机通信

ROS是一个分布式系统框架，因此有必要介绍一下分布式多机通信的设置方法。分布式一般都是主从（master/slave）方式，因此需要两端都要进行设置。

作为示例的两台机器如下：

- master: 192.168.1.100
- client: 192.168.1.200

master及client分别为各自的hostname, 想要查询自己的hostname也非常简单，就使用hostname命令。

#### Master端设置

- 修改hosts文件

```bash
sudo gedit /etc/hosts
```

添加：

```
192.168.1.100 master
192.168.1.200 client
```

#### Client端设置

- 修改hosts文件，方法与master相同

添加：

```
192.168.1.100 master
192.168.1.200 client
```

- 设置环境变量ROS_MASTER_URI

```bash
sudo gedit ~/.bashrc
```

添加

```
export ROS_MASTER_URI=http://192.168.1.100:11311
```

#### 测试

在master上打开roscore, 并发送一条消息

```bash
$ rostopic pub /test_topic std_msgs/String "connected"
```

在client上看是否能收到消息

```bash
$ rostopic echo /test_topic
data: "connected"
---
```



#### 总结

ros对分布式的支持是非常友好的，设置起来也非常方便，总结一下就两步：

- master及其它client都需要设置IP地址映射: 这一步不设置的话只能看到topic名称，但不能收到消息；
- client需要额外设置ROS_MASTER_URI