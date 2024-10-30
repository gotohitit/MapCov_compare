代码比较输入的两个点云地图的相似性，通过计算点云地图的协方差矩阵，计算相关性系数。



步骤 1：创建项目文件
    确保在一个文件夹中有以下两个文件：
    main.cpp - 上述代码文件。
    CMakeLists.txt - 用于构建的CMake配置文件。

步骤 2：创建编译目录并运行CMake
    为了保持项目目录整洁，创建一个 build 文件夹，存放编译文件和输出文件。

    在项目的根目录下打开终端，然后输入以下命令：
        mkdir build
        cd build
        cmake ..
    CMake 会检测 CMakeLists.txt，查找PCL和Eigen库并生成 Makefile。确保安装了PCL 和 Eigen库。如果缺少任何依赖库，可以使用以下命令安装（假设你使用的是Ubuntu）：
    函数库要求：
             eigen库3.3.7
             pcl库1.10

步骤 3：编译代码
    在 build 目录下运行以下命令以编译程序：
        make
    这会生成一个名为 PointCloudComparison 的可执行文件。

步骤 4：运行程序
编译完成后，你可以使用以下命令来运行程序：
    ./PointCloudComparison <file1.pcd> <file2.pcd> [downsample] [leaf_size]
    例如：
    ./PointCloudComparison ../CloudMap/GlobalMap1.pcd ../CloudMap/GlobalMap2.pcd 1 0.2

其中参数含义如下：  
    <file1.pcd>：第一个点云地图的文件路径。
    <file2.pcd>：第二个点云地图的文件路径。
    [downsample]：可选参数（0或1），指定是否进行降采样。1 表示启用降采样，0 表示不降采样。
    [leaf_size]：可选参数，指定体素网格大小（默认为0.1米）。仅在 downsample 参数为 1 时有效。