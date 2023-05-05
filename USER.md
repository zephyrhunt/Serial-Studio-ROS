# USER Develop

* 开发流程以及注意事项
* 关于该软件的使用和安装请查看WiKi

## 获取/初步编译工程

* fork 原始工程到https://github.com/zephyrhunt/Serial-Studio-ROS

* 克隆git clone --recursive https://github.com/zephyrhunt/Serial-Studio-ROS

* 创建该README.md文件，并完成一次提交**user develop init**。

* 编译工程
  ```bash
  qmake 
  make -j8
  ```

   注意 qmake 需要注意版本，如果之前安装过其他版本的QT，最好指定qmake绝对路径，如笔者为**/home/nichijou/Qt/6.3.2/gcc_64/bin/qmake**，查看其版本，项目在QT6.3.2下编译而成。
  
  ```bash
  $ /home/nichijou/Qt/6.3.2/gcc_64/bin/qmake --version
  QMake version 3.1
  Using Qt version 6.3.2 in /home/nichijou/Qt/6.3.2/gcc_64/lib
  ```
  
  编译结束后主目录下会现一个可执行文件**serial-studio**，执行该文件，可以打开该软件。
  
  ![image-20230505114633556](image/image-20230505114633556.png)

* 注意
  此时目录下将会有很多很乱的文件，我们现在把他们收到一起，现使用git完全重置到上一次提交。使用**QT Creator**打开进行编译，执行。原始的.pro文件中未添加qml，会有报错，添加即可。

  > QT += qml

## 工程概览

* 参考Serial-Studio [DOXGEN文档](https://serial-studio.github.io/hackers/)

主要关注src文件夹，其中

* **CSV文件夹:** 
  * 包含CSV::Export 和 CSV::Player
* **IO文件夹:** 
  * 包含IO::Manager, IO::DataSources::Serial, IO::DataSources::Network & IO::Console。
  * 其中IO::Manager & IO::Console是单例类，即只有一个实例，外部不会进行额外的实例化。
* **JSON文件夹**:
  * JSON::Frame包含Json的一些信息，frame对象的生成来自连接的设备和JSON映射文件。
  * JSON::Group一组对象。
  * JSON::Dataset数据集对象。
  * JSON::Generator处理来自IO::Manager的数据和JSON映射文件的数据，用来生成frame对象。
  * JSON::FrameInfo...
  * JSON::Editor允许用户在软件上编辑JSON文件。
* **Misc文件夹:**
  * 杂项类，包含一些实用的类。
* **MQTT文件夹**
  * 实现MQTT的功能。
* **Plugins文件夹**
  * 与外部插件交互。
* **UI文件夹**
  * 为QML提供数据的类，使得QML能够表示frame对象。
  * **Widgets文件夹**： 大部分widgets的数据显示都在此处实现。实现基于[Qwt](https://qwt.sourceforge.io/)
* 用户界面基于 QtQuick/QML，QML代码在./assets/qml文件夹中。
* 命名规则在.clang-format中，使用clang-format-all src格式化所有代码。

## 用户开发

### 开发目标

* 额外添加ROS交互
* 添加UI设置功能

### 开发记录

#### qmake转cmake

为了方便ROS的开发，需要把qmake工程转化为cmake工程。

参考脚本qmake2cmake https://www.qt.io/blog/introducing-qmake2cmake 阅读README.md使用

注意使用完脚本生成的CMakeList.txt还存在一点问题。

* 因为安装了ROS，电脑被提前安装了Qt5，需要在cmake中指定具体的Qt的目录。

  > ```cmake
  > set(CMAKE_PREFIX_PATH "/home/nichijou/Qt/6.3.2/gcc_64")
  > ```

* 根据报错对CMakeLists.txt做一些修改，例如RESOURCE ，注释原有的RESOURCE

  > ```cmake
  > qt_add_resources(RESOURCE assets/Resources.qrc)
  > ```

* 此判断会被默认判断为假 if(QWT_CONFIG_____contains_QwtPlot)，需要将其注释，否则会部分文件无法找到。

​		

