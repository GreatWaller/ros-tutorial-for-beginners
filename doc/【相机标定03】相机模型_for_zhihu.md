# 【相机标定03】相机模型--内参的组成

一般我们习惯用针孔相机模型来简化相机成像的过程。但是只有一个小孔在短暂曝光的情况下是不能获取足够的光线的，因此实际的相机通常都用透镜来折射更多的光线，但由此也带来了另一个问题--畸变。

故而，标定的目的，直观来讲，就是找出**三维空间中的物体在二维感光元件的投影位置**的计算模型。

我们从小孔成像说起：

![image-20211027113719968](https://raw.githubusercontent.com/GreatWaller/ros-tutorial-for-beginners/main/doc/images/image-20211027113719968.png)

根据相似三角形的基本推论，可知：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
-x/f &= \frac {X}{Z} \\
即,  -x &= f \cdot \frac {X}{Z} 
\end{align}\\
" alt="\begin{align}
-x/f &= \frac {X}{Z} \\
即,  -x &= f \cdot \frac {X}{Z} 
\end{align}\\
" class="ee_img tr_noresize" eeimg="1">
这个负号看起来很不舒服，我们把**Image plane 往前移至Pinhole plane**, 等效为：

![intrics](https://raw.githubusercontent.com/GreatWaller/ros-tutorial-for-beginners/main/doc/images/intrics.png)

即：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
x &= f \cdot \frac {X}{Z} ; &y= f \cdot \frac YZ \\

\end{align}\\
" alt="\begin{align}
x &= f \cdot \frac {X}{Z} ; &y= f \cdot \frac YZ \\

\end{align}\\
" class="ee_img tr_noresize" eeimg="1">
到目前为止，我们所有的单位都是长度，比如mm，我们要做的事情是把 <img src="https://www.zhihu.com/equation?tex=(x,y)" alt="(x,y)" class="ee_img tr_noresize" eeimg="1"> 转为像素表示 <img src="https://www.zhihu.com/equation?tex=(u,v)" alt="(u,v)" class="ee_img tr_noresize" eeimg="1"> 。

需要指出的是，pin hole 一般不是在成像平面的中心的，会有些许的偏移。习惯上，把光轴（optical axis）与 Image plane的交点的像素位置记为 <img src="https://www.zhihu.com/equation?tex=(u_0,v_0)" alt="(u_0,v_0)" class="ee_img tr_noresize" eeimg="1"> 。

那现在就有个问题： <img src="https://www.zhihu.com/equation?tex=x" alt="x" class="ee_img tr_noresize" eeimg="1"> 到底对应多少个像素呢？如果我们知道了 <img src="https://www.zhihu.com/equation?tex=\hat i" alt="\hat i" class="ee_img tr_noresize" eeimg="1"> 轴方向上 <img src="https://www.zhihu.com/equation?tex=x" alt="x" class="ee_img tr_noresize" eeimg="1"> 值对应的像素个数，那加上 <img src="https://www.zhihu.com/equation?tex=u_0" alt="u_0" class="ee_img tr_noresize" eeimg="1"> ，就得到了 <img src="https://www.zhihu.com/equation?tex=x" alt="x" class="ee_img tr_noresize" eeimg="1"> 的像素表示 <img src="https://www.zhihu.com/equation?tex=u" alt="u" class="ee_img tr_noresize" eeimg="1">  (**本例中像素坐标系的原点在左下角**，方向如上图所示)；同理可求出 <img src="https://www.zhihu.com/equation?tex=v" alt="v" class="ee_img tr_noresize" eeimg="1"> 。

假设在 <img src="https://www.zhihu.com/equation?tex=\hat i" alt="\hat i" class="ee_img tr_noresize" eeimg="1"> 方向上的像素长度为 <img src="https://www.zhihu.com/equation?tex=dx" alt="dx" class="ee_img tr_noresize" eeimg="1">  ，单位是 <img src="https://www.zhihu.com/equation?tex=mm /pixel" alt="mm /pixel" class="ee_img tr_noresize" eeimg="1"> , 表示每个像素的实际尺寸有多少毫米，因此 <img src="https://www.zhihu.com/equation?tex=x" alt="x" class="ee_img tr_noresize" eeimg="1"> 值的像素表示 <img src="https://www.zhihu.com/equation?tex=x_u" alt="x_u" class="ee_img tr_noresize" eeimg="1"> 为：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
x_u =y \cdot{1\over dx} &= f \cdot {1 \over dx } \cdot {X \over Z}
\end{align}\\
" alt="\begin{align}
x_u =y \cdot{1\over dx} &= f \cdot {1 \over dx } \cdot {X \over Z}
\end{align}\\
" class="ee_img tr_noresize" eeimg="1">
记  <img src="https://www.zhihu.com/equation?tex=f_x = f \cdot {1 \over dx}" alt="f_x = f \cdot {1 \over dx}" class="ee_img tr_noresize" eeimg="1"> ，因此 

<img src="https://www.zhihu.com/equation?tex=\begin{align}
x_u &= f_x \cdot {X \over Z}
\end{align}\\
" alt="\begin{align}
x_u &= f_x \cdot {X \over Z}
\end{align}\\
" class="ee_img tr_noresize" eeimg="1">
像素位置等于光心的坐标与相应方向上的偏移量之和，即：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
u &= f_x \cdot {X \over Z} + u_0 ; &v= f_y \cdot {Y \over Z} +v_0
\end{align}\\
" alt="\begin{align}
u &= f_x \cdot {X \over Z} + u_0 ; &v= f_y \cdot {Y \over Z} +v_0
\end{align}\\
" class="ee_img tr_noresize" eeimg="1">
最后，我们来写成矩阵的形式：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
Z
\left[
\begin{matrix}
u\\
v\\
1
\end{matrix}
\right]
&=
\left[
\begin{matrix}
f_x & 0 & u_0\\
0 & f_{y} & v_0\\
0 & 0 & 1
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
X\\
Y\\
Z\
\end{matrix}
\right]
= M\cdot
\left[
\begin{matrix}
X\\
Y\\
Z\
\end{matrix}
\right]
\end{align}\\
" alt="\begin{align}
Z
\left[
\begin{matrix}
u\\
v\\
1
\end{matrix}
\right]
&=
\left[
\begin{matrix}
f_x & 0 & u_0\\
0 & f_{y} & v_0\\
0 & 0 & 1
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
X\\
Y\\
Z\
\end{matrix}
\right]
= M\cdot
\left[
\begin{matrix}
X\\
Y\\
Z\
\end{matrix}
\right]
\end{align}\\
" class="ee_img tr_noresize" eeimg="1">

 <img src="https://www.zhihu.com/equation?tex=M" alt="M" class="ee_img tr_noresize" eeimg="1"> 就是所谓的相机内参。

#### 小结

总体说来，相机模型的推导还是很简单的，重要的是要了解内参的作用及其中每个参数的物理意义：

-  <img src="https://www.zhihu.com/equation?tex=f_x" alt="f_x" class="ee_img tr_noresize" eeimg="1"> : 焦距在 <img src="https://www.zhihu.com/equation?tex=u" alt="u" class="ee_img tr_noresize" eeimg="1"> 方向的像素表示
-  <img src="https://www.zhihu.com/equation?tex=f_y" alt="f_y" class="ee_img tr_noresize" eeimg="1"> : 焦距在 <img src="https://www.zhihu.com/equation?tex=v" alt="v" class="ee_img tr_noresize" eeimg="1"> 方向的像素表示
-  <img src="https://www.zhihu.com/equation?tex=u_0" alt="u_0" class="ee_img tr_noresize" eeimg="1"> : 光心所在位置的 <img src="https://www.zhihu.com/equation?tex=u" alt="u" class="ee_img tr_noresize" eeimg="1"> 方向像素坐标
-  <img src="https://www.zhihu.com/equation?tex=v_0" alt="v_0" class="ee_img tr_noresize" eeimg="1"> : 光心所在位置的 <img src="https://www.zhihu.com/equation?tex=v" alt="v" class="ee_img tr_noresize" eeimg="1"> 方向像素坐标

我们举个例子加深一下印象。以下是一个 <img src="https://www.zhihu.com/equation?tex=640 \times 480" alt="640 \times 480" class="ee_img tr_noresize" eeimg="1">  分辨率的相机内参：

<img src="https://www.zhihu.com/equation?tex=\left[
\begin{matrix}
5.3398795245975896e+02 & 0.& 3.2838647449406972e+02 \\
0. & 5.2871082110006125e+02 & 2.3684272831168110e+02 \\
0. & 0. & 1.
\end{matrix}
\right]\\
" alt="\left[
\begin{matrix}
5.3398795245975896e+02 & 0.& 3.2838647449406972e+02 \\
0. & 5.2871082110006125e+02 & 2.3684272831168110e+02 \\
0. & 0. & 1.
\end{matrix}
\right]\\
" class="ee_img tr_noresize" eeimg="1">
可见，它的光心就没有在正中心 <img src="https://www.zhihu.com/equation?tex=(320,240)" alt="(320,240)" class="ee_img tr_noresize" eeimg="1"> ，而是在  <img src="https://www.zhihu.com/equation?tex=(328,236)" alt="(328,236)" class="ee_img tr_noresize" eeimg="1">  处。

本节不再介绍畸变的相关内容，该部分相对独立，读者可自行查找资料学习。

学完了内参之后，再联系前两节的内容，我们就很容易总结出：

- **内参是用来将相机坐标投影成像素坐标**
- **外参是用来将世界坐标转换至相机坐标**

#### References

Adrian Kaehler, Gary Bradski - Learning OpenCV 3_ Computer Vision in C++ with the OpenCV Library (2017, O’Reilly Media) 

