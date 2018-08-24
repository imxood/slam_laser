# slam_learn

# slam及相关算法 学习笔记

~~ 部分代码来自深蓝学院, 互联网


## 发布里程计的过程:
    以10hz的频率从两个轮子读取当前速度Vr、Vl, 由差速控制得出变化值Dx、Dy、θ, 即位移translation(Dx、Dy), 旋转rotation(θ), 
    则位姿变量pose_delta(translation, rotation)

    更新位姿pos:
    pos.rot += rotation;
    pos.trans += rotationMatrix(pos.rot) + translation;

    关于旋转矩阵(下面使用3维坐标系):

    就是把小车(base_link坐标系)放在odom坐标系下, 实际上, 即时不偏移, 此时也存在不同系统之间的转换,
    转换矩阵为:
    1, 0, 0
    0, 1, 0
    0, 0, 1
    转换矩阵是由坐标系的单位向量组成的, 第1,2,3列分别是X,Y,Z轴的单位向量
    
    小车(所在坐标系为base_link坐标系)旋转θ角度, 此时小车坐标不会被改变, 但是小车坐标系发生了改变(旋转θ), 小车与odom坐标系就
    cos(θ), -sin(θ),  0
    sin(θ),  cos(θ),  0
    0     ,       0,  1

    此处旋转矩阵的理解: 旋转实际改变的是坐标轴, 与坐标无关, 你可以想象一下, 坐标系旋转了θ后, 矩阵第一列代表的是base_link的X轴,第二第三列,分别是YZ轴

    位姿信息更新完毕后, 会使用pos发布里程消息(odom_topic)及tf消息

    