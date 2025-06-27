基于定昌3588安装控制环境以及自启动步骤：
自启动方法：
1.sudo apt install gnome-startup-applications
2.gnome-session-properties
3.在弹出的自启动选项中添加，在命令栏中填入gnome-terminal即可
或
3.在命令栏中填入需要执行的命令，前面添加gnome-terminal -x“你所需的命令”即可
安装控制环境：
一.3588文件夹下GeographicLib-2.4文件（地理库）拷贝至ztl文件夹下
	1 进入/home/ztl/GeographicLib-2.4/BUILD/后删除所有文件
	2 sudo ../configure（提前设置/home/ztl/GeographicLib-2.4/下的configure权限为可执行或777） 
	3 sudo cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local 
	4 sudo make install
或
tar xfpz GeographicLib-2.4.tar.gz  //根据下载的库的版本号修改对于的版本
cd GeographicLib-2.4 
mkdir BUILD
cd BUILD  
../configure 
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local 
sudo make install
二.拷贝setup_can.sh到/home/ztl中，并更改权限为可执行文件
三.进入ztl下的.bashrc（默认隐藏，使用ctrl+H显示），在最后添加如下两行
	1 cd /home/ztl
	2 ./setup_can.sh
四.安装expect 解释器：sudo apt-get update
	sudo apt-get install expect
五.将mainControl_Test拷贝至/home/ztl，根据程序版本进行编译
六.将gps_path拷贝至/home/ztl，其中path.txt是控制读取路径的必要文件，需要根据情况更换
七.在主文件夹中创建log文件夹：mkdir log
八.GPS文件夹用于路径采集使用，使用相应GPS名称的文件夹内程序。（现有华测CGI610，博立3000P，INS821三款）
九.控制程序内Ipopt文件夹为二次规划库，作为MPC基础算法工具库使用
十.MPC_routine文件夹为MPC算法文件，视情况使用。使用时需要在mainControl_Test/CMakeLists.txt中进行更改


