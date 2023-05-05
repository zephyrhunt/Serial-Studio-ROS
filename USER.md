# USER Develop

* 开发流程以及注意事项

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
  此时目录下将会有很多很乱的文件，我们现在把他们收到一起，现使用git完全重置到上一次提交。
