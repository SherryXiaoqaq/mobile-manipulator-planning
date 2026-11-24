# 移动机械臂取放规划

《人工智能与自主机器人》Topic 2 课程项目。机器人从已知静态地图中的起点出发，停靠取货，携带货物绕障，再停靠交付。项目比较三种停车位选择策略，并增加行驶中同步收展的方案。

## 打开展示

下载仓库或 ZIP 并解压后，即可打开 PPT、离线回放和视频。

- [同步收展版 PPT](presentation/Topic2_移动机械臂取放规划_同步收展版.pptx) 是 14 页中文 PowerPoint。包含逆运动学、停车位选择、同步动作时序、实验对照和演讲者备注。
- 双击 [results/demo.html](results/demo.html) 在浏览器中离线播放。默认显示 **抓取侧遮挡** 场景的同步收展方案。可切换四个场景和四种方法，时间轴上的紫色区间表示底盘和机械臂同时运动。打开「同步收展算法」可比较两套时序，并逐个查看实际试过的候选时长和余隙下界。点击已采用窗口可定位回放。
- [results/simulation.mp4](results/simulation.mp4) 是抓取侧遮挡场景中协同规划的完整回放，便于在课堂上直接播放。
- [results/simulation_overlap.mp4](results/simulation_overlap.mp4) 是同一场景的同步收展回放。
- [results/metrics.csv](results/metrics.csv) 记录全部 16 组对照。 [results/experiment.json](results/experiment.json) 保存地图、路径、关节状态、货物位置、仿真时间戳和同步窗口。

## 运行代码

用 Python 3.10 或更高版本在项目根目录执行。规划和 HTML 生成仅使用 Python 标准库。

```powershell
python mobile_manipulator.py
python viz/build_demo.py
```

第一条命令会重写 `results/experiment.json` 和 `results/metrics.csv`。第二条把该 JSON 嵌入静态 HTML。若想保留原始实验，请用 `python mobile_manipulator.py --out results_rerun` 另存，再用 `python viz/build_demo.py --input results_rerun/experiment.json --output results_rerun/demo.html` 生成相应回放。

### 可选的视频重导出

安装 Node.js、Playwright 和 FFmpeg。FFmpeg 需支持 H.264 编码。默认重导出协同规划的遮挡场景。

```powershell
npm install --no-save playwright
npx playwright install chromium
node viz/export_video.cjs occluded_dock coupled
node viz/export_video.cjs occluded_dock overlap
```

如果 FFmpeg 不在 PATH 中，可通过 `TOPIC2_FFMPEG` 指定可执行文件路径。已安装 Edge 时也可设置 `TOPIC2_BROWSER_CHANNEL=msedge` 并使用该浏览器。

## 算法

1. 围绕取货和放置目标，在 0.80 m、1.00 m 两个距离各采样 16 个方向。把停车候选对齐 0.20 m 栅格，去重并过滤底盘不满足余隙条件的位置。
2. 以八邻域 A* 规划底盘路径。启发函数是到终点的欧氏距离。用底盘圆盘沿路径段扫掠的几何距离验证栅格边和路径捷径。
3. 对每个候选停车位计算平面两关节解析逆运动学。对两种弯肘方向尝试直接插值，也尝试保持折叠而转动肩关节再展开。展开与回收共用同一条关节路径，检查时始终计入末端货物的半径。 [2R 解析逆运动学](https://modernrobotics.northwestern.edu/nu-gm-book-resource/inverse-kinematics-of-open-chains/)
4. 机械臂用两条带半径的线段模拟，货物用圆盘模拟。对每段关节插值，采样的相邻位姿之间，任一几何点的位移上界至多 0.024 m。每个区间从端点采样的最小余隙减去半个区间的位移上界，得到保守的连续下界。底盘、机械臂和货物必须符合 0.06 m 余隙门槛。
5. 协同规划按候选取货位、交付位和机械臂动作组合枚举完整任务。取可行候选中评分最低的方案。随后生成底盘、关节和货物的时间序列，验证收臂行驶、碰撞余隙以及抓取与放置的末端位置。

令 L 为两段底盘总路程，A 为两个关节的累计绝对行程，c 为保守余隙下界。评分为 **J = L + 0.18 A + 0.20 / (0.15 + c)**。最近停靠先按直线距离选两个停车位，再检查任务可行性。逐段最短依次在可行候选中选每段行驶最短的停车位。三种方法共用几何、轨迹和安全检查。

### 进一步方案：同步收展

同步版先使用协同规划选好底盘路径和关节路径，再调整各段开始时间。取货前，伸臂与进站行驶的末段重叠。取货后，收臂与载货行驶的首段重叠。交付前，伸臂与载货行驶的末段重叠。抓取和释放各在停车状态保持 0.30 s。

每个窗口从允许的最大重叠开始尝试，然后逐步缩短。候选包括时间上限和 0.10 s 的整数倍。通过净空检查的第一个方案被采用。无法通过检查时，该段按原顺序执行。

用底盘位置和关节角折点对两条轨迹共同分段。每个线性区间内，任意连杆点或末端载荷点的位移上界为 `D = ‖Δb‖ + (l₁ + l₂)|Δq₁| + l₂|Δq₂|`。将区间分成 n 个采样子区间，从采样余隙扣除 `D/(2n)` 得到连续余隙下界，再与底盘精确扫掠余隙取最小值。保留余隙下界大于 0.06 m 的窗口。

CSV 中 `overlap` 行的 `score` 沿用停车位选择时的几何评分。同步版的时间改进见 `saved_time_s`，连续最小余隙下界见 `min_clearance_m`。按同一场景比较 `coupled` 和 `overlap` 即可。

`experiment.json` 的 `overlap_search` 记录逐窗口的候选尝试。通过项显示整个窗口的余隙下界。未通过项显示算法提前终止时的余隙下界，表示该候选未能通过余隙认证。

## 仿真结果

| 固定场景 | 最近停靠 | 逐段最短 | 协同规划 |
| --- | --- | --- | --- |
| 开放工位 | 完成 | 完成 | 完成 |
| 窄通道绕行 | 完成 | 完成 | 完成 |
| 抓取侧遮挡 | 取货位不可操作 | 完成 | 完成 |
| 通路封闭 | 底盘无路 | 底盘无路 | 底盘无路 |

- **抓取侧遮挡。** 逐段最短与协同规划的底盘路程分别为 12.22 m 和 11.00 m，少 10.0%。仿真执行时间分别为 21.12 s 和 19.37 s，少 8.3%。保守余隙下界分别为 0.068 m 和 0.062 m。
- **窄通道绕行。** 协同规划相对逐段最短多行驶 2.0%，关节累计行程减少 49.7%。协同规划选择了机械臂收展代价更低的组合。
- **复现。** 场景和算法均是确定性的。规划计算耗时依机器而异。仿真执行时间采用底盘 0.85 m/s、关节速度上限 1.65 rad/s 的时间参数化。

### 行驶中同步收展的耗时

| 固定场景 | 停车后收展 | 行驶中同步 | 节省 | 降幅 |
| --- | ---: | ---: | ---: | ---: |
| 开放工位 | 13.38 s | 10.54 s | 2.84 s | 21.2% |
| 窄通道绕行 | 19.44 s | 16.63 s | 2.81 s | 14.4% |
| 抓取侧遮挡 | 19.37 s | 15.60 s | 3.76 s | 19.4% |
| 通路封闭 | 底盘无路 | 底盘无路 | — | — |

三个成功场景的时间节省等于各自三个同步窗口的累计时长。认证最小余隙均达到 0.06 m 门槛。计算同步时序在本次运行中耗时约 3 到 33 ms，见 `retiming_time_ms`。

仿真假设底盘为固定朝向的全向平台，机械臂及障碍采用俯视二维建模，抓放以离散货物状态切换表示。接触动力学、机械臂自碰撞、定位误差、动态障碍和硬件跟踪属于后续验证范围。对照结果来自四个定向构造的场景和有限候选集。

## 资料

- Hart, Nilsson & Raphael (1968), *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*. [A* 原论文](https://ai.stanford.edu/~nilsson/OnlinePubs-Nils/PublishedPapers/astar.pdf)
- Lynch & Park (2017), *Modern Robotics*. [移动操作章节](https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-5-mobile-manipulation/)
- Lynch & Park (2017), *Modern Robotics*. [轨迹生成与时间参数化](https://modernrobotics.northwestern.edu/chapters/chapter9/)
- Xu et al. (2020), *Planning an Efficient and Robust Base Sequence for a Mobile Manipulator Performing Multiple Pick-and-place Tasks*. [论文页面](https://arxiv.org/abs/2001.08042)

封面概念插画见 [assets/cover.png](assets/cover.png)，使用内置图像生成工具。PPT 结果页的轨迹图由实验数据渲染。
