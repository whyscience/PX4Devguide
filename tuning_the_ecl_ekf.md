# 使用 ecl EKF

本节教程回答了使用 ECL EKF 算法相关的常见问题。

## 什么是 ecl EKF?

ECL \(Estimation and Control Library，估计和控制库\) 使用扩展卡尔曼滤波算法来处理传感器测量值并提供以下状态的估计:

* 四元数，定义了从North（北），East（东），Down（地）地球坐标系到X，Y，Z机体坐标系的旋转。
* IMU中的速度 - North,East,Down \(m/s\)
* IMU中的位置 - North,East,Down \(m\)
* IMU 角度增量偏差（bias）估计 - X,Y,Z \(rad\)
* IMU 速度增量偏差（bias）估计 - X,Y,Z\(m/s\)
* 地球磁场分量 - North,East,Down \(gauss\)
* 机体坐标系磁场偏差 - X,Y,Z \(gauss\)
* 风速 - North,East \(m/s\)

EKF在一个延迟的‘融合时间范围（fusion time horizon）’内工作，这是考虑到涉及到IMU的每个测量的时间延迟不同。每个传感器的数据是以FIFO（先进先出）形式缓存的，然后被EKF取出在正确的时间使用。EKF2\_\*\_DELAY 控制了每个传感器的延时补偿。

互补滤波用于将状态从‘融合时间范围’向当前时间推进，它使用IMU的数据。互补滤波的时间常数由参数EKF2\_TAU\_VEL 和 EKF2\_TAU\_POS控制。

注意：‘融合时间范围’的延迟和缓存的长度由 EKF2\_\*\_DELAY 中的最大值决定。如果一个传感器没有被使用，推荐将其时间延迟设置为0。降低‘融合时间范围’的延迟就降低了互补滤波中用于将状态推荐到当前时间的误差（error）。

位置和速度状态在被输出至控制回路之前，需要调整以说明IMU和机体坐标系的偏移（offset）。IMU相对于机体坐标系的位置可由参数 EKF2\_IMU\_POS\_X,Y,Z 设置。

EKF使用IMU仅用于状态预测。在EKF推到中IMU没有被用作观测量。协方差预测、状态更新和协方差更新的代数方程由 Matlab symbolic toolbox 生成，可以在这里找到: [Matlab Symbolic Derivation](https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m)

## 使用了什么传感器测量量？

EKF包含了不同的操作模式，考虑到了不同的传感器测量值得组合。在启动时，滤波器检测最小可用传感器组合，在初始倾斜、偏航和高度校准完成后，进入了一个提供旋转、垂直速度、垂直位置、IMU角度增量偏差和IMU速度增量偏离估计的模式。

这个模式需要IMU数据、一个偏航数据来源\(磁力计或者外部视觉信息\)和一个高度数据来源。所有EKF操作模式都需要这个最小数据集。其他传感器数据可用于估计其他的状态。

### IMU

* 机体三轴固连的惯性测量单元（Inertial Measurement unit）以最低100Hz提供角度增量和速度增量的数据。注意：在IMU角度增量数据被EKF使用之前应进行锥进修正。

### 磁力计

三轴机体固连的磁力计数据（或者外部视觉系统姿态数据）要求至少以5Hz更新。磁力计数据可用以下两种方式使用：

* 使用倾斜估计和磁力计偏差可将磁力计测量值转换为偏航角。这个偏航角接着被EKF用作观测量。这个方法更不准确而且没有考虑到机体坐标系磁场偏移，但是它对磁场异常和启动时大的陀螺仪偏差更加健壮（robust）。这是在启动时和在地面上时使用的默认方法。
* XYZ 磁力计读数被用做独立的观测量。这个方法更加准确而且允许习得机体坐标系磁场偏移，但是需假设地球磁场环境缓慢变化而且在有明显外部磁场干扰的时候表现欠佳。飞机在空中或者爬升超过1.5m高度之后默认使用这个方法。

选择模式的逻辑由参数 EKF2\_MAG\_TYPE 设置。

### 高度

一个高度数据源-至少以5Hz更新的GPS、气压、测距仪或者是外部视觉。注意：高度数据的主要来源受到参数 EKF2\_HGT\_MODE 的控制。

如果这些测量值不存在，EKF就不会启动。当这些测量值被检测到后，EKF会初始化状态并完成倾斜和偏航的校准。当倾斜和对其完成后，启用额外传感器的数据，EKF会接着转换到其他操作模式。

### GPS

如果以下条件满足，GPS测量值将会用于位置和速度：

* 启用GPS，通过设置参数 EKF2\_AID\_MASK。
* GPS质量检测通过。这些检测被参数 EKF2\_GPS\_CHECK and EKF2\_REQ&lt;&gt; 控制。
* GPS高度可以被EKF直接使用，通过设置参数 EKF2\_HGT\_MODE。

### 测距仪

测距仪到地面的距离被一个单独的状态估计使用来估计相对于高度基准的地形的垂直位置。

如果在一个可以当做零高度基准平面上操作，通过把参数 EKF2\_HGT\_MODE 设置为2，测距仪也可以直接被EKF使用用于估计高度。

### 风速

通过把 EKF2\_ARSP\_THR 设置为一个正值，等效风速（EAS）数据可用于估计风速数据和降低GPS丢失时的偏移。当风速数据超过 EKF2\_ARSP\_THR 的正值设定的阈值，而且飞机类型不是旋翼时，将启用风速数据。

### 合成侧滑（Synthetic Sideslip）

固定翼平台可以利用一个假设的零侧滑观测量来提升风速估计并在没有风速传感器的情况下启用风速估计。这可以通过把参 EKF2\_FUSE\_BETA 设置为1来使能。

### 光流

如果以下条件满足将使用光流数据：

* 有效的测距仪数据可用。
* 参数 EKF2\_AID\_MASK 的比特位1为true。
* 光流传感器返回的公制质量大于参数 EKF2\_OF\_QMIN 设定的最小要求。

###外部视觉系统

例如 Vicon这样的外部视觉系统的位置和姿态测量在以下状况下可用：

* 外部视觉系统的水平位置数据将启用，如果参数 EKF2\_AID\_MASK 的比特位3是为true。
* 外部视觉系统的垂直位置数据将启用，如果参数 EKF2\_HGT\_MODE 设置为3。
* 外部视觉系统的姿态数据将启用，如果参数 EKF2\_AID\_MASK 的比特位4是为true。

## 如何使用 'ecl' library EKF?

将参数 SYS\_MC\_EST\_GROUP 设置为2来使用 ecl EKF。

## ecl EKF 相比于其他估计器来说的优点和缺点是什么？

与所有的估计器一样，性能的大部分来自于匹配传感器特性的调试。调试是精度与鲁棒性的折中，尽管我们尝试提供满足大部分用户需求的调试参数，还是有有一些应用场合需要调试。 

因此没有，没有关于有关之前 attitude\_estimator\_q + local\_position\_estimator 这种组合的精度的声明保证，估计器的最佳选择取决于应用和调谐。

### 劣势

* ecl EKF 是一个复杂的算法，要求对扩展卡尔曼滤波和在导航问题上的应用有很好地理解，才能成功的进行调试。对于那些没有取得较好结果的用户而言，知道修改什么参数就更困难了。
* ecl EKF 使用更多的RAM和flash空间。
* ecl EKF 使用更多的日志空间。
* ecl EKF 飞行测试时间更短。

### 优势

* ecl EKF 能够在数学上连续地融合来自传感器的不同时间延迟和不同速率的数据，一旦延迟参数正确设置之后就能提升动态操控时的精度。
* ecl EKF 能够大范围的不同的传感器类型。
* ecl EKF 检测和报告传感器数据中的统计意义上明显的不一致性，协助诊断传感器误差。
* 对于固定翼运算，ecl EKF 有没有风速传感器都能估计风速，能够联合使用估计风速与风速计测量值和侧滑假设，进而延长飞行中 GPS 丢失时的航位推测时间。
* ecl EKF 估计三轴加速度计的偏差，能提升tailsitter和其他在飞行阶段之间会经历大的姿态变化的飞行器的精度。
* 这种联合结构（混合姿态和位置/速度估计）意味着姿态估计受益于所有传感器的测量。如果调试得当，这应该有提升姿态估计的潜力。

## 如何检测 EKF 性能？

EKF 输出、状态（state）和状态（status）数据被发布到一系列在飞行中记录到SD卡的 uORB 话题中。以下指导假设数据已经被以 .ulog 的文件格式记录。要使用 .ulog 格式，将参数 SYS\_LOGGER 设置为1.

.ulog格式的数据可以通过使用 [PX4 pyulog library](https://github.com/PX4/pyulog) 在python中解析。

大部分 EKF 数据可以在记录到 .ulog 文件中的 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 和 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) uORB 消息中找到。

### 输出数据

* 姿态输出数据在 [vehicle\_attitude](https://github.com/PX4/Firmware/blob/master/msg/vehicle_attitude.msg) 消息中。
* 本地位置输出在 [vehicle\_local\_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_local_position.msg) 消息中。
* 控制回路反馈数据在 [control\_state](https://github.com/PX4/Firmware/blob/master/msg/control_state.msg) 消息中。
* 全局 \(WGS-84\) 输出数据在 [vehicle\_global\_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_global_position.msg) 消息中。
* 风速输出数据在 [wind\_estimate](https://github.com/PX4/Firmware/blob/master/msg/wind_estimate.msg) mess消息中。

### 状态


参考 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 中的ststatesate\[32\]。 states\[32\] 的索引图如下:

* \[0 ... 3\] 四元数
* \[4 ... 6\] 速度 NED \(m/s\)
* \[7 ... 9\] 位置 NED \(m\)
* \[10 ... 12\] IMU 角度增量偏差 XYZ \(rad\)
* \[13 ... 15\] IMU 速度增量偏差 XYZ \(m/s\)
* \[16 ... 18\] 地球磁场 NED \(gauss\)
* \[19 ... 21\] 机体磁场 XYZ \(gauss\)
* \[22 ... 23\] 风速 NE \(m/s\)
* \[24 ... 32\] 未使用

### 状态方差

参考 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 中的covariances\[28\]（协方差）。 covariances\[28\]的索引图如下:

* \[0 ... 3\] 四元数
* \[4 ... 6\] 速度 NED \(m/s\)^2
* \[7 ... 9\] 位置 NED \(m^2\)
* \[10 ... 12\] IMU 角度增量偏差 XYZ \(rad^2\)
* \[13 ... 15\] IMU 速度增量偏差 XYZ \(m/s\)^2
* \[16 ... 18\] 地球磁场 NED \(gauss^2\)
* \[19 ... 21\] 机体磁场 XYZ \(gauss^2\)
* \[22 ... 23\] 风速 NE \(m/s\)^2
* \[24 ... 28\] 未使用

### 观测新息（Observation Innovations）

* 磁力计 XYZ \(gauss\) : 参考 mag\_innov\[3\] ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 偏航角 \(rad\) : 参考 heading\_innov ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* 速度和位置新息 : 参考 vel\_pos\_innov\[6\] ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。vel\_pos\_innov\[6\] 的索引图如下:
  * \[0 ... 2\] 速度 NED \(m/s\)
  * \[3 ... 5\] 位置 NED \(m\)
* 真实风速 \(m/s\) : 参考 airspeed\_innov ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 合成侧滑 \(rad\) : 参考 beta\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 光流 XY \(rad/sec\) : 参考 flow\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 地面上方高度 \(m\) : 参考 hagl\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。

### 观测信息方差

* 磁力计 XYZ \(gauss^2\) : 参考 mag\_innov\_var\[3\] ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 偏航角 \(rad^2\) : 参考 heading\_innov\_var，位于the ekf2\_innovations message.
* 速度和位置新息 : 参考 vel\_pos\_innov\_var\[6\]，位于[ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。 vel\_pos\_innov\[6\] 的索引图如下:
  * \[0 ... 2\] 速度 NED \(m/s\)^2
  * \[3 ... 5\] 位置 NED \(m^2\)
* 真实风速 \(m/s\)^2 : 参考 airspeed\_innov\_var ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 合成侧滑 \(rad^2\) : 参考 beta\_innov\_var ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 光流 XY \(rad/sec\)^2 : 参考 flow\_innov\_var ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。
* 地面上方高度 \(m^2\) : 参考 hagl\_innov\_var ，位于 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)。

### 输出互补滤波

输出互补滤波用于将状态从融合时间范围推进到当前时间。要查看融合时间范围内测量的角度、速度和位置追踪误差量级（magnitude），参考 ekf2\_innovations 中的 output\_tracking\_error\[3\]索引图如下：
* \[0\] 角度追踪误差量级 \(rad\)
* \[1\] 速度追踪误差量级 \(m/s\)。 可以通过调整参数 EKF2\_TAU\_VEL来调整速度追踪时间常数。降低这个参数会降低稳态误差，但是会增加NED速度输出的观测噪声数量。
* \[2\] 位置追踪误差量级 \(m\)。   可以通过调整参数 EKF2\_TAU\_POS来调整位置追踪时间常数。降低这个参数会降低稳态误差，但是会增加NED位置输出的观测噪声数量。

### EKF 误差

EKF对于糟糕状态下的状态和协方差更新内置了误差检测。 参考 filter\_fault\_flags ，位于 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg)。

### 观测误差

有两种类型的观测错误:

* 数据丢失。一个例子就是测距仪不能提供返回值。
* 新息，就是状态估计和传感器观测之间的差异超出了。一个例子就是过多的震动导致大的垂直位置误差，导致气压高度测量值被拒绝。

这些都会导致观测数据被拒绝足够长时间而导致EKF尝试使用传感器观测值重置状态。所有的观测都有一个统计置信度检测应用于新息。检测的标准差的数目受到每种观测类型的参数 EKF2\_&lt;&gt;\_GATE 的控制。


测试水平（Test levels）在 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 中，如下所示:

* mag\_test\_ratio : 最大磁力计新息分量与新息测试限制之比
* vel\_test\_ratio : 最大速度新息分量与新息测试限制之比
* pos\_test\_ratio : 最大水平位置新息分量与新息测试限制之比
* hgt\_test\_ratio : 最大垂直位置新息分量与新息测试限制之比
* tas\_test\_ratio : 最大真实风速新息分量与新息测试限制之比
* hagl\_test\_ratio : 最大地面上方高度新息分量与新息测试限制之比

对于每个传感器的二进制 pass/fail 汇总, 参考 innovation\_check\_flags ，位于 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg)。

### GPS 质量检测

开始GPS辅助之前，EKF进行了一系列的GPS质量检测。这些检测由参数 EKF2\_GPS\_CHECK 和 EKF2\_REQ&lt;&gt;控制。这些检测的 pass/fail 状态记录在 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).gps\_check\_fail\_flags 消息中。当所有要求的GPS检测通过，这个整数将为0。如果EKF没有开始GPS校准，查看 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 中的 gps\_check\_fail\_flags 位掩码的定义。

### EKF 数值误差

为了降低对处理器的要求，对所有的运算EKF使用单精度浮点类型，对协方差预测和更新方程使用一阶近似。这意味着当重新调试EKF时有可能遇到异常情况，在其中协方差矩阵操作条件变得很糟糕以至于导致状态估计中产生发散或者明显的误差。

为了阻止这个，每一个协方差和状态更新步骤包含以下误差检测和修正步骤：

* 如果新息方差小于观测方差（这要求一个负的状态方差，这是不可能的）或者协方差更新将对任何状态产生负的方差：
  * 状态和协方差更新被跳过
  * 协方差矩阵中相应的行和列被重置
  * 失败被记录在 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) filter\_fault\_flags 消息中
* 状态方差 \(协方差矩阵中的对角线\) 被限制成非负值
* 上限被应用到状态方差中去
* 协方差矩阵强制对称

重新调试滤波器后，像降低噪声变量、estimator\_status.gps\_check\_fail\_flags 的数值这样的部分重新调试应该再次检测以确保仍然为零。

## 如果高度估计发散该怎么办？

EKF高度在飞行中远离GPS和高度计测量值，最常见原因是震动导致的IMU限幅（clipping）和/或混淆（aliasing）。如果种种情况出现，以下迹象在数据中应该很明显：

* [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).vel\_pos\_innov\[3\] and  [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).vel\_pos\_innov\[5\] 都将有同样的迹象。
* [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).hgt\_test\_ratio 将大于1.0

推荐第一步使用一个有效的隔离安装系统确保飞控与机架隔离。一个隔离底座具有6个自由度，因此有6个共振频率。作为通用规则，安装在隔离底座上的飞控的的6个共振频率应该大于25Hz以避免与飞控动力学的交叉，并且低于点击的频率。

如果共振频率与点击或螺旋桨的转动频率重合，隔离底座只会使得振动情况更加糟糕。

通过The EKF can be made more resistant to vibration induced height divergence by making the following parameter changes:

* Double the value of the innovation gate for the primary height sensor. If using barometeric height this is EK2\_EKF2\_BARO\_GATE.
* Increase the value of EKF2\_ACC\_NOISE to 0.5 initially. If divergence is still occurring,   increase in further increments of 0.1 but do not go above 1.0

Note that the effect of these changes will make the EKF more sensitive to errors in GPS vertical velocity and barometric pressure.

## What should I do if the position estimate is diverging?

The most common causes of position divergence are:

* High vibration levels. 
  * Fix by improving mechanical isolation of the autopilot.
  * Increasing the value of EKF2\_ACC\_NOISE and EKF2\_GYR\_NOISE can help, but does make the EKF more vulnerable to GPS glitches.
* Large gyro bias offsets. 
  * Fix by re-calibrating the gyro. Check for excessive temperature sensitivity \(&gt; 3 deg/sec bias change during warm-up from a cold start and replace the sensor if affected of insulate to to slow the rate of temeprature change.
* Bad yaw alignment
  * Check the magntometer calibration and alignment.
  * Check the heading shown QGC is within within 15 deg truth
* Poor GPS accuracy
  * Check for interference
  * Improve separation and shielding
  * Check flying location for GPS signal obstructions and reflectors \(nearboy tall buildings\)
* Loss of GPS

Determining which of these is the primary cause requires a methodical approach to analysis of the EKF log data:

* Plot the velocty innovation test ratio - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vel\_test\_ratio
* Plot the horizontal position innovation test ratio - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg)。pos\_test\_ratio
* Plot the height innovation test ratio - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).hgt\_test\_ratio
* Plot the magnetoemrer innovation test ratio - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).mag\_test\_ratio
* Plot the GPS receier reported speed accuracy - [vehicle\_gps\_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_gps_position.msg).s\_variance\_m\_s
* Plot the IMU delta angle state estimates - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).states\[10\], states\[11\] and states\[12\]
* Plot the EKF internal high frequency vibration metrics:
  * Delta angle coning vibration - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[0\]
  * High frequency delta angle vibration - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[1\]
  * High frequency delta velocity vibration - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[2\]

During normal operation, all the test ratios should remain below 0.5 with only occasional spikes above this as shown in the example below from a successful flight:  
![Position, Velocity, Height and 磁力计 Test Ratios](Screen Shot 2016-12-02 at 9.20.50 pm.png)  
The following plot shows the EKF vibration metrics for a multirotor with good isolation. The landing shock and the increased vibration during takeoff and landing can be seen. Insufficient data has been gathered with these metrics to provide specific advice on maximum thresholds.  
![](Screen Shot 2016-12-02 at 10.24.00 pm.png)  
The above vibration metrics are of limited value as the presence of vibration at a frequency close to the IMU sampling frequency \(1 kHz for most boards\) will cause  offsets to appear in the data that do not show up in the high frequency vibration metrics. The only way to detect aliasing errors is in their effect on inertial navigation accuracy and the rise in innovation levels.

In addition to generating large position and velocity test ratios of &gt; 1.0, the different error mechanisms affect the other test ratios in different ways:

### Determination of Excessive Vibration

High vibration levels normally affect vertical position and velocity innovations as well as the horizontal components. 磁力计 test levels are only affected to a small extent.

\(insert example plots showing bad vibration here\)

### Determination of Excessive Gyro Bias

Large gyro bias offsets are normally characterised by a change in the value of delta angle bias greater than 5E-4 during flight \(equivalent to ~3 deg/sec\) and can also cause a large increase in the 磁力计 test ratio if the yaw axis is affected. Height is normally unaffected other than extreme cases. Switch on bias value of up to 5 deg/sec can be tolerated provided the filter is given time time settle before flying . Pre-flight checks performed by the commander should prevent arming if the position is diverging.

\(insert example plots showing bad gyro bias here\)

### Determination of Poor Yaw Accuracy

Bad yaw alignment causes a velocity test ratio that increases rapidly when the vehicle starts moving due inconsistency in the direction of velocity calculated by the inertial nav and the  GPS measurement. 磁力计 innovations are slightly affected. Height is normally unaffected.

\(insert example plots showing bad yaw alignment here\)

### Determination of Poor GPS Accuracy

Poor GPS accuracy is normally accompanied by a rise in the reported velocity error of the receiver in conjunction with a rise in innovations. Transient errors due to multipath, obscuration and interference are more common causes. Here is an example of a temporary loss of GPS accuracy where the multi-rotor started drifting away from its loiter location and had to be corrected using the sticks. The rise in [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vel\_test\_ratio to greater than 1 indicates the GPs velocity was inconsistent with other measurements and has been rejected.

![](gps glitch - test ratios.png)

This is accompanied with rise in the GPS receivers reported velocity accuracy which indicates that it was likely a GPS error.  
![](gps glitch - reported receiver accuracy.png)

If we also look at the GPS horizontal velocity innovations and innovation variances, we can see the large spike in North velocity innovation that accompanies this GPS 'glitch' event.  
![](gps glitch - velocity innovations.png)

### Determination of GPS Data Loss

Loss of GPS data will be shown by the velocity and position innvoation test ratios 'flat-lining'. If this occurs, check the oher GPS status data in vehicle\_gps\_position for further information.

\(insert example plots showing loss of GPS data here\)

