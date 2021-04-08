# ACFLY-Modified

20200103更新内容：

1、磁场异常检测、GPS航向对准，无惧磁场干扰
2、全部传感器融合带健康检测，GPS等被干扰不会导致炸鸡（干扰严重会切回定高）
3、姿态控制算法更新，收敛更柔和，可以限制期望角速度、角加速度
4、位置控制算法更新，收敛更柔和
5、SD卡驱动可用，持续记录下次更新，可自己写数据

20200404更新内容：
1、健康度接口信息完善
2、修正串口+DMA发送卡住的bug
3、支持经纬度坐标二维、三维飞行
3、MPU bug修复
4、支持SD卡记录

20200420更新内容：
1、修正SD低速卡数据误码的问题（32库函数bug）
2、连接地面站自动修正RTC时间

20200423更新内容：
！！注意！！从老版本升级到此版本及之后版本需要重新注册和全部重新校准
！！注意！！注册码请问群主拿
1、支持航点规划飞行（测试中，支持MP和QGC地面站航点规划）

20200505更新内容：
1、航点规划飞行稳定版本
2、支持电压检测，根据电压动态调整控制参数，电压电量回传
3、位置解算改进减小悬停波动


20200524更新内容：
1、增加位置控制参数，可以调节高度控制器响应速度
2、高度控制自适应（防止力气过大的机子抖震）
3、支持修改自动飞行速度
4、地面站姿态显示bug修正
5、显示屏刷新卡bug修正


20200610（beta）更新内容：
1、高度控制相位超前降噪（解决部分机子抖震）
2、加快罗盘异常情况下航向对准速度（解决航向不准一直绕圈圈问题）
3、位置控制改进，手感提升n倍
4、位置解算改进，增加收敛稳定性，降低噪声敏感程度
20200610正式版更新内容：
1、按钮3一键返航（校准的第三个按钮）
2、mini光流支持
3、支持限制最大转动速度，适应大飞机（AC_MaxRPSp AC_MaxYSp参数，
	大飞机TD4P1可以相应调小，穿越机可以调大可以非常快速）
4、支持更改扰动跟随速度（AC_beta2参数），扰动跟随越快抗干扰越强
	但是抗震动越差
5、支持限制手动飞行和自动飞行的最大飞行速度、加速度、加加速度（PC里面的参数）
6、Beta版本小bug修正
已知问题：
1、陀螺零偏较大时，一上电起飞有可能飘得比较严重，可以上电后稍微等一会，下一版本解决。

20200617更新内容：
1、解算快速零偏纠正算法，解决陀螺零偏造成位置漂移问题，定点更稳定。
2、解锁后电机按顺序开始转动
3、油门最下关电机强制保护改为 油门最下+偏航最右 保护，油门最低电机不会停转
4、地面站可显示卫星数量，飞控模式

20200701更新内容：
1、解算+控制小bug修复
2、支持qgc一键起飞执行任务

20200728更新内容：
1、姿态、高度解算更新（抗扰更快，稳定性更好）
	！！姿态参数重置！！请记录原本b T参数方便重新调整
2、解算小bug修复

20200817更新内容：
1、解决姿态控制器容易震颤问题
	！！将AC_Beta默认参数修改为12，请自行调整！！
2、解决纯气压（无GPS）容易掉高的bug，纯气压暴力飞行基本不掉高
3、对GPS等速度位置传感器融合进行优化，GPS定点更稳定
4、传感器根据信任度及噪声自主切换，更加智能，自主切换GPS、光流等传感器
5、航线飞行支持调速，支持测绘定距拍照

20200922-Beta更新内容：
1、解决高度容易漂移，绝对高度不准问题
2、增加SD读卡器功能
3、修复gps等低采样率传感器延时补偿不准确导致刹车漂移问题

20201106更新内容：
1、支持65536个航点
2、支持飞控和位置传感器安装偏移补偿（GPS等）
3、支持地面站图形化校准，支持按钮功能设置
4、支持低电量自动返航降落
5、增加uwb驱动（替换光流）
6、解决测距传感器毛刺容易造成掉高的问题

20201231更新内容：
！！注意！！此次升级会刷掉包括注册码的所有配置
1、内置flash更换带掉电保护和写入均衡的文件系统，解决flash存储丢失问题
2、提升航线飞行速度、转弯速度、返航速度
3、支持RTK基站数据注入（mission planner RTK注入页面）
4、优化解算系统稳定性（高度更稳定）
5、支持外接PX4LED（iic接口自动识别）
6、修正模式bug

20200226更新内容：
1、提升断点（断电）续飞稳定性
2、支持通过参数配置每个串口功能
3、支持通过SD卡更新bootloader