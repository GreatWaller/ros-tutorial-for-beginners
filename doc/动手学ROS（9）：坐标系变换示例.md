# 动手学ROS（9）：坐标系变换示例

本节是上一篇《动手学ROS（8）：坐标系变换基础》的代码示例。

#### 两个坐标系间的变换

$P_o=R \cdot P_c + \vec t $ 

```c++
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
int main(int argc, char const *argv[])
{
    Eigen::Quaterniond q(0.35, 0.2, 0.3, 0.1);
    q.normalize();
    Eigen::Vector3d T(0.3, 0.1, 0.1);

    Eigen::Vector3d p_c(0.5, 0, 0.2);
    // 也可 auto p_c = q * p_c + T;
    auto p_o = q.matrix() * p_c + T;
    std::cout << p_o << std::endl;

    return 0;
}

```

使用Eigen库，可以方便地操作四元数及矩阵的乘法。`q.matrix() `可得到一个$3\times3$的旋转矩阵$R$。

#### 多个坐标系间的转换

$P_b=T_b^{-1} \cdot T_c \cdot P_c$

引用《视觉SLAM十四讲》第三章中的示例：相对于世界坐标系的1号坐标系(位姿 $q1, t1$)和2号坐标系$(q2, t2)$，求1中一点$p1$在2号坐标系下的坐标。

```c++
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char** argv) {
  Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
  q1.normalize();
  q2.normalize();
  Eigen::Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);
  Eigen::Vector3d p1(0.5, 0, 0.2);

  Eigen::Isometry3d T1w(q1), T2w(q2);
  T1w.pretranslate(t1);
  T2w.pretranslate(t2);

  // 原书：Eigen::Vector3d p2 = T2w * T1w.inverse() * p1;
  Eigen::Vector3d p2=T2w.inverse() * T1w * p1;
  std::cout << p2.transpose() << std::endl;
  return 0;
}
```

这里需要指出，原书作者对于坐标系位姿的相对关系定义可能与我是相反的，故他使用 $p2 = T2w * T1w.inverse() * p1$,我们理解即可。

#### 小结

本节仅仅是对上节的辅助示例，主要内容详见《动手学ROS（8）：坐标系变换基础》。
