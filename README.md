基于双目立体视觉的三维重建
==========================
本科毕业论文，[点击这里下载论文](http://cooljiansir.oss-cn-beijing.aliyuncs.com/github/%E6%9B%BE%E5%BF%97%E5%88%9A-%E5%9F%BA%E4%BA%8E%E5%8F%8C%E7%9B%AE%E7%AB%8B%E4%BD%93%E8%A7%86%E8%A7%89%E7%9A%84%E4%B8%89%E7%BB%B4%E9%87%8D%E5%BB%BAv3.1.pdf)

## 三维重建的步骤：
三维重建首先需要对两个摄像头进行标定：
* 单目标定摄像头（左）
* 单目标定摄像头（右）
* 双目标定

标定之后，对于双目摄像头拍摄到的两张图片A,B需要经过如下步骤进行三维重建：
* 极线矫正
* 双目匹配
* 三维重建

本论文最终以三维点云的方式重建。

## 软件说明
下载链接：[点击这里](http://cooljiansir.oss-cn-beijing.aliyuncs.com/github/3dreconstruction.zip)
其中`3dreconstruction.exe`是32位windows可执行程序。

**菜单说明**

**双目摄像头参数标定：**

* File :打开/新建/保存 双目摄像头标定参数文件`.dre`
* Single :单目标定（包含畸变矫正）
* Binocular :双目标定

**双目图片重建**

* Polar Correction :极线矫正
* Stereo Match :立体匹配
* 3d reconstruction :opengl 三维点云重建

## 快速浏览
![](https://raw.githubusercontent.com/cooljiansir/3dreconstruction/master/quicklook.gif)
