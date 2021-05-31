# 动手学ROS（8）：坐标系间的欧氏变换

之前的文章仅仅是让我们可以run起来ros程序，没有涉及到任何实际的应用场景，从今天开始我们会渐渐接触。

本节的内容不多同时也比较直观，但曾经也觉得有些困扰，在经历多次计算错误后，才决定彻底把坐标变换搞明白。

阅读本节需要一些前置知识：欧氏变换、旋转向量、四元数，这些都是基本的数学概念，希望读者自行学习，这里不再赘述。

#### 两个坐标系间的变换

在开始具体地讲解之前，需要搞清楚两个特别容易混淆的概念：

- 坐标系的变换
- 点在坐标系间的变换

前者表示两个坐标系的坐标轴之间的变换关系；后者说的是一个坐标系的点在另一个坐标系中如何表示。本节会多次提到这两种说法，请读者注意。

请看下图，我们先把这两个概念搞清楚：



![tf](https://raw.githubusercontent.com/GreatWaller/ros-tutorial-for-beginners/main/doc/images/tf.png)

从坐标系**Object**（以下简称 <img src="https://www.zhihu.com/equation?tex=C_o" alt="C_o" class="ee_img tr_noresize" eeimg="1"> ）到坐标系**Camera**（以下简称 <img src="https://www.zhihu.com/equation?tex=C_c" alt="C_c" class="ee_img tr_noresize" eeimg="1"> ）的变换为 <img src="https://www.zhihu.com/equation?tex=(R,\vec t)" alt="(R,\vec t)" class="ee_img tr_noresize" eeimg="1">  ，这就是坐标系间的变换，R表示旋转矩阵， <img src="https://www.zhihu.com/equation?tex=\vec t" alt="\vec t" class="ee_img tr_noresize" eeimg="1"> 表示平移向量。

 <img src="https://www.zhihu.com/equation?tex=P_c" alt="P_c" class="ee_img tr_noresize" eeimg="1"> 是 <img src="https://www.zhihu.com/equation?tex=C_c" alt="C_c" class="ee_img tr_noresize" eeimg="1"> 中一点，求该点在 <img src="https://www.zhihu.com/equation?tex=C_o" alt="C_o" class="ee_img tr_noresize" eeimg="1"> 中的坐标 <img src="https://www.zhihu.com/equation?tex=P_o" alt="P_o" class="ee_img tr_noresize" eeimg="1"> ，这就是点在坐标系间的变换。

图中已经给出了计算方法， <img src="https://www.zhihu.com/equation?tex=P_o=R \cdot P_c + \vec t " alt="P_o=R \cdot P_c + \vec t " class="ee_img tr_noresize" eeimg="1">  。这个式子本身没有什么可说的，这里提供一个辅助记忆的方法：

> 假设在 <img src="https://www.zhihu.com/equation?tex=C_o" alt="C_o" class="ee_img tr_noresize" eeimg="1"> 中有一点 <img src="https://www.zhihu.com/equation?tex=P_f" alt="P_f" class="ee_img tr_noresize" eeimg="1"> ，它的坐标与 <img src="https://www.zhihu.com/equation?tex=P_c" alt="P_c" class="ee_img tr_noresize" eeimg="1"> 相对 <img src="https://www.zhihu.com/equation?tex=C_c" alt="C_c" class="ee_img tr_noresize" eeimg="1"> 一样，同为 <img src="https://www.zhihu.com/equation?tex=(x,y,z)" alt="(x,y,z)" class="ee_img tr_noresize" eeimg="1"> ，那坐标系 <img src="https://www.zhihu.com/equation?tex=C_o" alt="C_o" class="ee_img tr_noresize" eeimg="1"> 通过 <img src="https://www.zhihu.com/equation?tex=(R,\vec t)" alt="(R,\vec t)" class="ee_img tr_noresize" eeimg="1"> 变换到 <img src="https://www.zhihu.com/equation?tex=C_c" alt="C_c" class="ee_img tr_noresize" eeimg="1"> 后， <img src="https://www.zhihu.com/equation?tex=P_f" alt="P_f" class="ee_img tr_noresize" eeimg="1"> 将与 <img src="https://www.zhihu.com/equation?tex=P_c" alt="P_c" class="ee_img tr_noresize" eeimg="1"> 重合，即为要求的 <img src="https://www.zhihu.com/equation?tex=P_o" alt="P_o" class="ee_img tr_noresize" eeimg="1"> 。

#### 多个坐标系间的转换

我们只举一个三个坐标系间转换的例子，三个以上同理。

![3_transform](https://raw.githubusercontent.com/GreatWaller/ros-tutorial-for-beginners/main/doc/images/3_transform.png)

如上图，我们在刚才例子的基础上增加一个坐标系 <img src="https://www.zhihu.com/equation?tex=C_b" alt="C_b" class="ee_img tr_noresize" eeimg="1"> ，已知坐标系 <img src="https://www.zhihu.com/equation?tex=C_c" alt="C_c" class="ee_img tr_noresize" eeimg="1"> 和 <img src="https://www.zhihu.com/equation?tex=C_b" alt="C_b" class="ee_img tr_noresize" eeimg="1"> 相对于世界坐标系 <img src="https://www.zhihu.com/equation?tex=C_o" alt="C_o" class="ee_img tr_noresize" eeimg="1"> 的转换关系， <img src="https://www.zhihu.com/equation?tex=P_c" alt="P_c" class="ee_img tr_noresize" eeimg="1"> 是 <img src="https://www.zhihu.com/equation?tex=C_c" alt="C_c" class="ee_img tr_noresize" eeimg="1"> 中一点，求该点在 <img src="https://www.zhihu.com/equation?tex=C_b" alt="C_b" class="ee_img tr_noresize" eeimg="1"> 中的坐标 <img src="https://www.zhihu.com/equation?tex=P_b" alt="P_b" class="ee_img tr_noresize" eeimg="1"> 。为了书写方便，坐标之间的变换采用变换矩阵 <img src="https://www.zhihu.com/equation?tex=T" alt="T" class="ee_img tr_noresize" eeimg="1"> 表示。变换矩阵也就是把rotation和translation写成齐次的形式。

我们使用刚才的公式连续转换就可以了，第一步转到 <img src="https://www.zhihu.com/equation?tex=C_o(T_c \cdot P_c)" alt="C_o(T_c \cdot P_c)" class="ee_img tr_noresize" eeimg="1"> ，然后再转到 <img src="https://www.zhihu.com/equation?tex=C_b" alt="C_b" class="ee_img tr_noresize" eeimg="1"> 。第二步的时候，我们需要知道从B到O的变换矩阵， <img src="https://www.zhihu.com/equation?tex=T_b" alt="T_b" class="ee_img tr_noresize" eeimg="1"> 求逆，即 <img src="https://www.zhihu.com/equation?tex=T_b^{-1}" alt="T_b^{-1}" class="ee_img tr_noresize" eeimg="1"> 。最后再乘起来 <img src="https://www.zhihu.com/equation?tex=P_b=T_b^{-1} \cdot T_c \cdot P_c" alt="P_b=T_b^{-1} \cdot T_c \cdot P_c" class="ee_img tr_noresize" eeimg="1"> 。

这里，我们可以再换个角度来看下上面的公式， <img src="https://www.zhihu.com/equation?tex=T_b^{-1} \cdot T_c" alt="T_b^{-1} \cdot T_c" class="ee_img tr_noresize" eeimg="1"> 其实就是坐标系B到C的变换，最后再画个图来强化记忆：





![b2c](https://raw.githubusercontent.com/GreatWaller/ros-tutorial-for-beginners/main/doc/images/b2c.png)

> 留意箭头的方向， <img src="https://www.zhihu.com/equation?tex=B->C=B->C \cdot O->C" alt="B->C=B->C \cdot O->C" class="ee_img tr_noresize" eeimg="1"> 。类似于向量的加法，不过次序一定要保持从头到尾的指向。
>

#### 小结

本节仅是提供了一个更Intuitive的方式来理解和记忆坐标系间的转换，这样写代码时就不容易出错。后续会提供常用的代码示例。

