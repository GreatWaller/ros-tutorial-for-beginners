# 【相机标定07】求解相机内参

运用上一节《【相机标定06】求解单应矩阵》中的方法，我们就可以求解出单应矩阵，本节就继续介绍如何从单应矩阵中分解出内参。

我们从单应矩阵的定义入手：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
H &= \left[\begin{matrix}\vec h_1 & \vec h_2 & \vec h_3\end{matrix} \right] = s \cdot M \cdot \left[\begin{matrix}\vec r_1 & \vec r_2 & \vec t\end{matrix} \right]
\end{align}
" alt="\begin{align}
H &= \left[\begin{matrix}\vec h_1 & \vec h_2 & \vec h_3\end{matrix} \right] = s \cdot M \cdot \left[\begin{matrix}\vec r_1 & \vec r_2 & \vec t\end{matrix} \right]
\end{align}
" class="ee_img tr_noresize" eeimg="1">
由上式可知：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
\vec h_1 &= s \cdot M \cdot \vec r_1\\
\vec h_2 &= s \cdot M \cdot \vec r_2\\
\vec h_3 &= s \cdot M \cdot \vec t\\
\end{align}
" alt="\begin{align}
\vec h_1 &= s \cdot M \cdot \vec r_1\\
\vec h_2 &= s \cdot M \cdot \vec r_2\\
\vec h_3 &= s \cdot M \cdot \vec t\\
\end{align}
" class="ee_img tr_noresize" eeimg="1">
故而：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
\vec r_1 &= \lambda \cdot M^{-1} \cdot \vec h_1\\
\vec r_2 &= \lambda \cdot M^{-1} \cdot \vec h_2\\
\vec t &= \lambda \cdot M^{-1} \cdot \vec h_3\\
\end{align}
" alt="\begin{align}
\vec r_1 &= \lambda \cdot M^{-1} \cdot \vec h_1\\
\vec r_2 &= \lambda \cdot M^{-1} \cdot \vec h_2\\
\vec t &= \lambda \cdot M^{-1} \cdot \vec h_3\\
\end{align}
" class="ee_img tr_noresize" eeimg="1">
其中  <img src="https://www.zhihu.com/equation?tex=\lambda = {1 \over s}" alt="\lambda = {1 \over s}" class="ee_img tr_noresize" eeimg="1"> ，又  <img src="https://www.zhihu.com/equation?tex=\vec r_1" alt="\vec r_1" class="ee_img tr_noresize" eeimg="1">  和  <img src="https://www.zhihu.com/equation?tex=\vec r_2" alt="\vec r_2" class="ee_img tr_noresize" eeimg="1">  正交，正交向量的内积为0：

<img src="https://www.zhihu.com/equation?tex={\vec r_1} ^{T} \cdot \vec r_2 = 0
" alt="{\vec r_1} ^{T} \cdot \vec r_2 = 0
" class="ee_img tr_noresize" eeimg="1">
 代入  <img src="https://www.zhihu.com/equation?tex=\vec r_1" alt="\vec r_1" class="ee_img tr_noresize" eeimg="1">  和  <img src="https://www.zhihu.com/equation?tex=\vec r_2" alt="\vec r_2" class="ee_img tr_noresize" eeimg="1">  ，得到第一个**约束条件1**:

<img src="https://www.zhihu.com/equation?tex=\begin{align}
(\lambda \cdot M^{-1} \cdot \vec h_1)^T \cdot (\lambda \cdot M^{-1} \cdot \vec h_2) &= 0 \\
\lambda ^2 \cdot {\vec h_1}^T \cdot ({M^{-1}})^T \cdot M^{-1} \cdot \vec h_2 &=0 \\
{\vec h_1}^T \cdot ({M^{-1}})^T \cdot M^{-1} \cdot \vec h_2 &=0 \tag{1}\\
\end{align}
" alt="\begin{align}
(\lambda \cdot M^{-1} \cdot \vec h_1)^T \cdot (\lambda \cdot M^{-1} \cdot \vec h_2) &= 0 \\
\lambda ^2 \cdot {\vec h_1}^T \cdot ({M^{-1}})^T \cdot M^{-1} \cdot \vec h_2 &=0 \\
{\vec h_1}^T \cdot ({M^{-1}})^T \cdot M^{-1} \cdot \vec h_2 &=0 \tag{1}\\
\end{align}
" class="ee_img tr_noresize" eeimg="1">
我们继续寻找另一个**约束条件2**， <img src="https://www.zhihu.com/equation?tex=\vec r_1" alt="\vec r_1" class="ee_img tr_noresize" eeimg="1">  和  <img src="https://www.zhihu.com/equation?tex=\vec r_2" alt="\vec r_2" class="ee_img tr_noresize" eeimg="1">  是旋转矩阵中两个正交分向量，因此他们的模长也是相等的：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
|| \vec r_1|| &= || \vec r_2 ||\\
{\vec r_1} ^{T} \cdot \vec r_1 &= {\vec r_2} ^{T} \cdot \vec r_2\\
{\vec h_1}^T \cdot ({M^{-1}})^T \cdot M^{-1} \cdot \vec h_1 &= {\vec h_2}^T \cdot ({M^{-1}})^T \cdot M^{-1} \cdot \vec h_2 \tag{2}\\
\end{align}
" alt="\begin{align}
|| \vec r_1|| &= || \vec r_2 ||\\
{\vec r_1} ^{T} \cdot \vec r_1 &= {\vec r_2} ^{T} \cdot \vec r_2\\
{\vec h_1}^T \cdot ({M^{-1}})^T \cdot M^{-1} \cdot \vec h_1 &= {\vec h_2}^T \cdot ({M^{-1}})^T \cdot M^{-1} \cdot \vec h_2 \tag{2}\\
\end{align}
" class="ee_img tr_noresize" eeimg="1">
设 

<img src="https://www.zhihu.com/equation?tex=B = ({M^{-1}})^T \cdot M^{-1} = \left[
\begin{matrix}
B_{11} & B_{12} & B_{13}\\
B_{21} & B_{22} & B_{23}\\
B_{31} & B_{32} & B_{33}\\
\end{matrix}
\right]
" alt="B = ({M^{-1}})^T \cdot M^{-1} = \left[
\begin{matrix}
B_{11} & B_{12} & B_{13}\\
B_{21} & B_{22} & B_{23}\\
B_{31} & B_{32} & B_{33}\\
\end{matrix}
\right]
" class="ee_img tr_noresize" eeimg="1">

从内参 $M = \left[
\begin{matrix}
f_x & 0 & c_{x} \\
0 & f_{y} & c_{y} \\
0 & 0 & 1 
\end{matrix}
\right]$ 入手，可通过简单的初等变换求解出其逆矩阵:

<img src="https://www.zhihu.com/equation?tex=M^{-1}=
\left[\begin{matrix}
1 \over f_x & 0 & -{c_{x} \over f_x }\\
0 & 1 \over f_{y} & -{c_{y} \over f_y} \\
0 & 0 & 1 
\end{matrix}
\right]
" alt="M^{-1}=
\left[\begin{matrix}
1 \over f_x & 0 & -{c_{x} \over f_x }\\
0 & 1 \over f_{y} & -{c_{y} \over f_y} \\
0 & 0 & 1 
\end{matrix}
\right]
" class="ee_img tr_noresize" eeimg="1">
代入到  <img src="https://www.zhihu.com/equation?tex=B" alt="B" class="ee_img tr_noresize" eeimg="1">  中：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
B &= ({M^{-1}})^T \cdot M^{-1}\\
&=
\left[\begin{matrix}
1 \over f_x & 0 & 0\\
0 & 1 \over f_{y} & 0 \\
-{c_{x} \over f_x } & -{c_{y} \over f_y} & 1 
\end{matrix}
\right]
\cdot
\left[\begin{matrix}
1 \over f_x & 0 & -{c_{x} \over f_x }\\
0 & 1 \over f_{y} & -{c_{y} \over f_y} \\
0 & 0 & 1 
\end{matrix}
\right]\\
&=
\left[\begin{matrix}
1 \over {f_x}^2 & 0 & -{c_{x} \over {f_x}^2 }\\
0 & 1 \over {f_y}^2 & -{c_{y} \over {f_y}^2} \\
-{c_x \over {f_x}^2} & -{c_y \over {f_y}^2} & \left( {{c_x}^2 \over {f_x}^2} +{{c_y}^2 \over {f_y}^2}  +1 \right) 
\end{matrix}
\right]\\
\end{align}
" alt="\begin{align}
B &= ({M^{-1}})^T \cdot M^{-1}\\
&=
\left[\begin{matrix}
1 \over f_x & 0 & 0\\
0 & 1 \over f_{y} & 0 \\
-{c_{x} \over f_x } & -{c_{y} \over f_y} & 1 
\end{matrix}
\right]
\cdot
\left[\begin{matrix}
1 \over f_x & 0 & -{c_{x} \over f_x }\\
0 & 1 \over f_{y} & -{c_{y} \over f_y} \\
0 & 0 & 1 
\end{matrix}
\right]\\
&=
\left[\begin{matrix}
1 \over {f_x}^2 & 0 & -{c_{x} \over {f_x}^2 }\\
0 & 1 \over {f_y}^2 & -{c_{y} \over {f_y}^2} \\
-{c_x \over {f_x}^2} & -{c_y \over {f_y}^2} & \left( {{c_x}^2 \over {f_x}^2} +{{c_y}^2 \over {f_y}^2}  +1 \right) 
\end{matrix}
\right]\\
\end{align}
" class="ee_img tr_noresize" eeimg="1">

##### 求解B

接下来只要求出了矩阵B，我们就自然可以求得内参。

首先，我们观察B中的元素，发现B是一个对称矩阵。其实一个矩阵和自身的转置相乘的结果就是对称矩阵。因此我们可以减少B中待求的元素个数至6个（保留上三角的数字）：

<img src="https://www.zhihu.com/equation?tex=B = 
\left[
\begin{matrix}
B_{11} & B_{12} & B_{13}\\
B_{12} & B_{22} & B_{23}\\
B_{13} & B_{23} & B_{33}\\
\end{matrix}
\right]
" alt="B = 
\left[
\begin{matrix}
B_{11} & B_{12} & B_{13}\\
B_{12} & B_{22} & B_{23}\\
B_{13} & B_{23} & B_{33}\\
\end{matrix}
\right]
" class="ee_img tr_noresize" eeimg="1">


然后，我们重写上述的两个结束条件：

<img src="https://www.zhihu.com/equation?tex=\left\{
\begin{aligned} 
{\vec h_1}^T \cdot B \cdot \vec h_2 &=0\\
{\vec h_1}^T \cdot B \cdot \vec h_1 &= {\vec h_2}^T \cdot B \cdot \vec h_2 \\
\end{aligned} 
\right.
\tag{3}
" alt="\left\{
\begin{aligned} 
{\vec h_1}^T \cdot B \cdot \vec h_2 &=0\\
{\vec h_1}^T \cdot B \cdot \vec h_1 &= {\vec h_2}^T \cdot B \cdot \vec h_2 \\
\end{aligned} 
\right.
\tag{3}
" class="ee_img tr_noresize" eeimg="1">
观察可知，我们需要计算   <img src="https://www.zhihu.com/equation?tex={\vec h_i}^T \cdot B \cdot \vec h_j" alt="{\vec h_i}^T \cdot B \cdot \vec h_j" class="ee_img tr_noresize" eeimg="1">  这样的一般形式。

这里符号比较多，再明确一下： <img src="https://www.zhihu.com/equation?tex=\vec h" alt="\vec h" class="ee_img tr_noresize" eeimg="1">  是单应矩阵的列向量， <img src="https://www.zhihu.com/equation?tex=\vec h_i = \left[ \begin{matrix} h_{i,1} \\ h_{i,2} \\ h_{i,3} \end{matrix} \right]" alt="\vec h_i = \left[ \begin{matrix} h_{i,1} \\ h_{i,2} \\ h_{i,3} \end{matrix} \right]" class="ee_img tr_noresize" eeimg="1">  。代入展开：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
{\vec h_i}^T \cdot B \cdot \vec h_j &=
\left[ \begin{matrix} h_{i,1} & h_{i,2} & h_{i,3} \end{matrix} \right] 
\cdot
\left[
\begin{matrix}
B_{11} & B_{12} & B_{13}\\
B_{12} & B_{22} & B_{23}\\
B_{13} & B_{23} & B_{33}\\
\end{matrix}
\right]
\cdot
\left[ \begin{matrix} h_{j,1} \\ h_{j,2} \\ h_{j,3} \end{matrix} \right] \\
&=
h_{i,1} h_{j,1} B_{11} +
\left( h_{i,1} h_{j,2} +h_{i,2} h_{j,1} \right)B_{12} +
\left( h_{i,1} h_{j,3} +h_{i,3} h_{j,1} \right)B_{13} +
h_{i,2} h_{j,2} B_{22} +
\left( h_{i,2} h_{j,3} +h_{i,3} h_{j,2} \right)B_{23} +
h_{i,3} h_{j,3} B_{33}\\
&=
{\left[
\begin{matrix}
h_{i,1} h_{j,1}\\
h_{i,1} h_{j,2} +h_{i,2} h_{j,1}\\
h_{i,1} h_{j,3} +h_{i,3} h_{j,1}\\
h_{i,2} h_{j,2}\\
h_{i,2} h_{j,3} +h_{i,3} h_{j,2}\\
h_{i,3} h_{j,3} \\
\end{matrix}
\right]}^T
\cdot
\left[
\begin{matrix}
B_{11}\\
B_{12}\\
B_{13}\\
B_{22}\\
B_{23}\\
B_{33}\\
\end{matrix}
\right]\\
&=
{\vec v_{i,j}}^T
\cdot
\vec b
\end{align}
" alt="\begin{align}
{\vec h_i}^T \cdot B \cdot \vec h_j &=
\left[ \begin{matrix} h_{i,1} & h_{i,2} & h_{i,3} \end{matrix} \right] 
\cdot
\left[
\begin{matrix}
B_{11} & B_{12} & B_{13}\\
B_{12} & B_{22} & B_{23}\\
B_{13} & B_{23} & B_{33}\\
\end{matrix}
\right]
\cdot
\left[ \begin{matrix} h_{j,1} \\ h_{j,2} \\ h_{j,3} \end{matrix} \right] \\
&=
h_{i,1} h_{j,1} B_{11} +
\left( h_{i,1} h_{j,2} +h_{i,2} h_{j,1} \right)B_{12} +
\left( h_{i,1} h_{j,3} +h_{i,3} h_{j,1} \right)B_{13} +
h_{i,2} h_{j,2} B_{22} +
\left( h_{i,2} h_{j,3} +h_{i,3} h_{j,2} \right)B_{23} +
h_{i,3} h_{j,3} B_{33}\\
&=
{\left[
\begin{matrix}
h_{i,1} h_{j,1}\\
h_{i,1} h_{j,2} +h_{i,2} h_{j,1}\\
h_{i,1} h_{j,3} +h_{i,3} h_{j,1}\\
h_{i,2} h_{j,2}\\
h_{i,2} h_{j,3} +h_{i,3} h_{j,2}\\
h_{i,3} h_{j,3} \\
\end{matrix}
\right]}^T
\cdot
\left[
\begin{matrix}
B_{11}\\
B_{12}\\
B_{13}\\
B_{22}\\
B_{23}\\
B_{33}\\
\end{matrix}
\right]\\
&=
{\vec v_{i,j}}^T
\cdot
\vec b
\end{align}
" class="ee_img tr_noresize" eeimg="1">
我们再次重写约束（3）：

<img src="https://www.zhihu.com/equation?tex=\left[
\begin{matrix}
v_{12}^T\\
v_{11}^T-v_{22}^T\\
\end{matrix}
\right]
\cdot
\vec b
=0 \tag{4}
" alt="\left[
\begin{matrix}
v_{12}^T\\
v_{11}^T-v_{22}^T\\
\end{matrix}
\right]
\cdot
\vec b
=0 \tag{4}
" class="ee_img tr_noresize" eeimg="1">
这样一张图片就可以列出两个方程，那6个未知量就**至少需要3张图片**。

如果你观察仔细的话， <img src="https://www.zhihu.com/equation?tex=B_{12}" alt="B_{12}" class="ee_img tr_noresize" eeimg="1">  在上面的推导中其实为0，那是因为我们一直以来介绍的内参是没有考虑制造过程中产生的坐标轴偏差，实际中的内参还有一个参数，导致  <img src="https://www.zhihu.com/equation?tex=B_{12}" alt="B_{12}" class="ee_img tr_noresize" eeimg="1">  不为0，但值往往也非常小。在忽略这个偏差的情况下，要求的变量就只剩下5个，再考虑到上一节中介绍的求解单应矩阵的方法，可以提出一个因子  <img src="https://www.zhihu.com/equation?tex=\lambda" alt="\lambda" class="ee_img tr_noresize" eeimg="1">  ，使  <img src="https://www.zhihu.com/equation?tex=B_{33}" alt="B_{33}" class="ee_img tr_noresize" eeimg="1">  为1，这样就只剩下4个未知量，那仅需2张图片即可求解  <img src="https://www.zhihu.com/equation?tex=B" alt="B" class="ee_img tr_noresize" eeimg="1">  。不过图片当然多一点更好。

##### 内参

在求解出  <img src="https://www.zhihu.com/equation?tex=B" alt="B" class="ee_img tr_noresize" eeimg="1">  之后，就可以解出内参了：

<img src="https://www.zhihu.com/equation?tex=\left\{
\begin{aligned} 
f_x &= \sqrt {\lambda \over B_{11}} \\
f_y &= \sqrt {\lambda \over B_{22}} \\
c_x &= - \left( B_{13}f_x^2 \over \lambda \right) = - \left( B_{13} \over B_{11} \right)\\
c_y &= - \left( B_{23}f_y^2 \over \lambda \right) = - \left( B_{23} \over B_{22} \right)\\
\lambda &= B_{33} - {B_{13}^2 \over B_{11}^2} - {B_{23}^2 \over B_{22}^2}
\end{aligned} 
\right.
\tag{5}
" alt="\left\{
\begin{aligned} 
f_x &= \sqrt {\lambda \over B_{11}} \\
f_y &= \sqrt {\lambda \over B_{22}} \\
c_x &= - \left( B_{13}f_x^2 \over \lambda \right) = - \left( B_{13} \over B_{11} \right)\\
c_y &= - \left( B_{23}f_y^2 \over \lambda \right) = - \left( B_{23} \over B_{22} \right)\\
\lambda &= B_{33} - {B_{13}^2 \over B_{11}^2} - {B_{23}^2 \over B_{22}^2}
\end{aligned} 
\right.
\tag{5}
" class="ee_img tr_noresize" eeimg="1">
由于我们没有考虑内参中的坐标轴偏差，这里的结果比较容易计算。当然实际中是要考虑的，这里不再赘述。笔者所有的文章为了易读易复现都会做适当的简化，读者也应以理解核心思想为目的。理解之后，需要的就是动手实践。

##### 外参

有了内参，外参就自然可以算出：

<img src="https://www.zhihu.com/equation?tex=\left\{
\begin{align}
\vec r_1 &= \lambda \cdot M^{-1} \cdot \vec h_1\\
\vec r_2 &= \lambda \cdot M^{-1} \cdot \vec h_2\\
\vec r_3 &= \vec r_1 \cdot \vec r_2\\
\vec t &= \lambda \cdot M^{-1} \cdot \vec h_3\\
\end{align}
\right.
\tag{6}
" alt="\left\{
\begin{align}
\vec r_1 &= \lambda \cdot M^{-1} \cdot \vec h_1\\
\vec r_2 &= \lambda \cdot M^{-1} \cdot \vec h_2\\
\vec r_3 &= \vec r_1 \cdot \vec r_2\\
\vec t &= \lambda \cdot M^{-1} \cdot \vec h_3\\
\end{align}
\right.
\tag{6}
" class="ee_img tr_noresize" eeimg="1">

##### 小结

到这里，求解的主要过程已经推导完毕，但该算法的后续处理仍是十分重要的，比如旋转矩阵的的各个列向量两两正交，且模为1，这都提供了更加严格的约束，这里就不做更加详细的介绍了。

##### 参考

Adrian Kaehler, Gary Bradski - Learning OpenCV 3_ Computer Vision in C++ with the OpenCV Library (2017, O’Reilly Media) 

