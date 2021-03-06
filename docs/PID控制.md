# PID控制

我们将PID控制分成了三个模式：

1. 尽量走过道的中线，即我们在小作业3中使用的PID算法
2. 遇到障碍物，不能再走中线，要倒车绕障
3. 调整路线，即扭头

## 待机时

如果我们已经到达了上一个终点，或者我们刚刚开机，此时小车不该进行移动，而应该进行待机。（判断方法为`tarx == 0 && tary == 0`）

此时默认输出的`speed`和`steer`均为0。

直到用户发出信号，小车才开始向下一个目标点前进。

## 准备出发时

准备出发时，我们需要确定方向，这个可以参考[如何寻找目标.md](./如何寻找目标.md)，如果方向不对，那么我们利用模式3转向。

## 行动时

默认模式为1。

如果检测到前方有障碍物，那么我们将模式修改为2。在认为我们越过了障碍物之后，我们将模式修改回1。

## 到达终点时

在通过检测，认为我们到达了终点的时候，我们发出信号并恢复待机。