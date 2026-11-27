# Clevo 风扇控制指示器

适用于 Ubuntu 的 Clevo（蓝天）笔记本风扇控制工具，在系统托盘显示温度和风扇信息，支持自动调速和手动控制。

本仓库：https://github.com/ldbsmh/clevo-indicator

原项目地址：https://github.com/SkyLandTW/clevo-indicator

---

## 更新日志

### 2026-11-26（曲线重写：节点插值 + 双热源 + 滞回）

**新增**
- 自动曲线改为「温度:占空比」节点表 + 线性插值，节点可在 `~/.config/clevo-indicator/curve` 自定义（2–10 个，温度递增、占空比不回退）
- CPU / GPU 独立曲线，每轮取需求更大的一方，GPU 真发热时也会参与升速
- 临界温度锁存：≥ 92°C 强制全速，回落到 86°C 才交还曲线
- `clevo-indicator --curve` 打印解析结果与插值表，改配置后可直接核对

**修复**
- 旧算法的「瞬时升温 + 6 秒均值降温」两路信号在温度斜坡上互相打架，风扇在相邻两档之间反复跳（26%→31%→26%）；改为单一控制温度（升立即跟随、降 1/8 衰减、最多滞后 4°C）
- 降档后立刻被重新判为「该升了」，同一温度连写三次 EC（25%→22%→24%）；升的门槛改为大于降的单步幅度

### 2026-11-23（GPU 温度 + 温度悬浮窗）

**新增**
- GPU 温度改由 NVIDIA 驱动接口（NVML）提供：本机 EC 的 `0xCD` 恒为 0（混合显卡机型不映射 GPU 二极管），现在通过 `libnvidia-ml.so.1` 直接读取，实测可正常显示（空闲约 29°C）
- 桌面悬浮小卡片：一屏看清 CPU/GPU 温度、当前风扇档位百分比、双风扇转速
- 卡片无边框、圆角半透明、始终置顶、不抢焦点；拖动可移动，位置与开关状态持久化到 `~/.config/clevo-indicator/overlay`
- 卡片上右键弹出与托盘同一份菜单；双击打开滑块调节窗口

### 2026-11-22（本机适配）

**新增**
- 托盘菜单增加「风扇：自定义…」，弹出滑块窗口，可在 15% ~ 100% 之间按 1% 连续调节，并实时显示实际占空比与转速
- 手动档增加高温安全网：占空比过低而温度顶到 93°C 时临时全速，降到 80°C 以下恢复用户设定

**调整**
- 手动档预设从 60/70/80/90/100 改为 20/30/40/50/70/100，下限从 60% 降到 15%（实测 10% 仍不失速，留 5% 余量）
- 自动调速曲线按本机实测重标：原曲线以「空闲 30°C」为前提，本机空闲就有 45–52°C，导致自动模式长期停在 70–80% 降不下来；现在空闲约 28–36%

### 2026-03-12

**新增**
- GPU 风扇控制：现在同时控制 CPU 风扇（EC 端口 `0x01`）和 GPU 风扇（EC 端口 `0x02`），两个风扇始终保持相同转速
- 新增 GPU 风扇转速读取（寄存器 `0xD2/0xD3`）
- 托盘图标动画改为 CPU 和 GPU 风扇转速的平均值

**修复**
- 自动调速模式静默失败：原最小值校验为 60%，但自动算法可能返回 30/40/50%，导致风扇转速无法降低，现已修复
- `MAX_FAN_RPM` 从 4400 调整为 5500，与实际硬件匹配

### 历史版本

- 适配 Ubuntu 22+：从 `appindicator3` 迁移至 `ayatana-appindicator3`

---

## 功能

- 系统托盘实时显示 CPU 和 GPU 温度
- 托盘图标动态反映风扇转速（转速越高图标越满）
- 自动调速模式：根据 CPU/GPU 温度自动调节风扇占空比，取两者较高温度为基准
- 手动模式：托盘菜单提供 20/30/40/50/70/100% 预设，另有「自定义」窗口可用滑块连续调节 15% ~ 100%
- 桌面悬浮卡片：实时显示 CPU/GPU 温度、当前档位百分比、双风扇转速，可拖动摆位
- CPU 风扇与 GPU 风扇始终同步，统一控制
- CPU 电源档位切换：菜单里可直接改写 `intel_pstate` 的 `energy_performance_preference`（省电 / 保持系统默认）

## 托盘菜单

菜单只有一个，打开后上下分区（托盘协议只暴露一个菜单入口，左右键由桌面环境统一指向它，无法做左右分工）：

```
● CPU 电源：省电（推荐）      ← 当前生效项用 ● 标记
○ CPU 电源：保持系统默认
──────────────────────────
● 风扇：自动（按温度）
──────────────────────────
○ 风扇：20%（静音）
○ 风扇：30%
○ 风扇：40%
○ 风扇：50%
○ 风扇：70%
○ 风扇：100%（全速）
○ 风扇：自定义…
──────────────────────────
○ 悬浮窗：显示温度与转速
```

点「自定义…」会弹出一个带滑块的小窗口，可在 `15% ~ 100%` 之间按 1% 连续调节，窗口内实时显示实际占空比与转速（见下节）。

菜单里**没有退出项**：程序由 systemd 托管常驻，停止只能通过 `systemctl --user stop clevo-indicator`（见下节）。

`CPU 电源` 只有两档，因为本机实测 `balance_power` / `balance_performance` / `performance` 三者行为完全相同（空闲时也锁 3.9GHz），只有 `power` 能真正降频，所以不提供没有实际区别的「平衡」档。

选择会写入 `~/.config/clevo-indicator/epp`，下次启动自动恢复。写 sysfs 需要 root，由 worker 子进程执行，UI 只通过共享内存发请求。

**EPP 自愈**：worker 每 2 秒核对一次 `energy_performance_preference`。发现被外部改掉会立刻写回用户的选择，并记一条日志（含时间、改动前后值、当时 governor 与频率）：

```bash
sudo journalctl -t clevo-indicator | grep 电源偏好
```

没有这一步时，旧版本只是「回读并跟随显示」——值一旦被别的程序或固件改掉，菜单就诚实显示成「保持系统默认」，而且永远不会纠正，看起来就像「息屏久了自动变回默认」。

> worker 进程已 `setuid(0)`，它的日志归到 root 名下，因此要用系统日志查，`journalctl --user` 里看不到。

## 悬浮窗

托盘图标只能显示两个温度，看不到转速和档位，所以另有一块常驻的小卡片：

```
CPU   48°C         GPU   29°C
风扇  32% 自动     转速  2480/2400 RPM
```

- **档位**是 EC 回读的实际 PWM 占空比（`0xCE` 寄存器），自动模式下也照实显示，能直接看出自动曲线当前把风扇放在哪一档
- **CPU 温度**取自 EC 寄存器 `0x07`
- **GPU 温度**优先取 EC 寄存器 `0xCD`；该寄存器在混合显卡（Optimus）机型上恒为 0，此时自动改用 NVIDIA 驱动接口（NVML）读取，两者都拿不到才显示 `--`
- **转速**依次为 CPU 风扇 / GPU 风扇（寄存器 `0xD0/D1`、`0xD2/D3`）
- 温度着色：< 75°C 灰白，≥ 75°C 琥珀，≥ 88°C 红
- **拖动**移动位置，松手即保存；**右键**弹出与托盘相同的菜单；**双击**打开滑块调节窗口
- 窗口 `accept_focus = FALSE`，点它不会把正在使用的窗口挤到后面
- 开关状态与坐标存在 `~/.config/clevo-indicator/overlay`（第一行 1/0 表示显示/隐藏，第二行坐标），上次开着的话下次启动自动恢复

SNI（StatusNotifierItem）协议只能渲染普通菜单项，放不下滑块和实时读数，因此这两项功能都用独立窗口实现，而不是塞进菜单。

## 自动调速算法

曲线不再是写死的档位表，而是一张「温度 : 占空比」节点表，节点之间线性插值（配置里可切成阶梯跳变）。CPU 与 GPU 各有一条曲线，每一轮分别算一次，取需要更大的那个——两个风扇共用同一个占空比，哪个热源更热就按哪个吹。

| 温度 | CPU 曲线 | GPU 曲线 | 实际取用 |
|------|---------|---------|---------|
| 40°C | 18% | 18% | 18% |
| 50°C | 23% | 20% | 23% |
| 60°C | 34% | 27% | 34% |
| 70°C | 50% | 45% | 50% |
| 80°C | 73% | 75% | 75% |
| 84°C | 84% | 87% | 87% |
| 88°C | 96% | 100% | 100% |
| ≥ 90°C | 100% | 100% | 100% |

默认节点（可通过配置文件改）：

```
cpu 40:18 48:22 56:28 64:40 72:54 78:68 84:84 89:100
gpu 40:18 55:22 62:30 70:45 76:60 80:75 85:90 88:100
```

CPU 曲线按本机实测标定（空闲 46–50°C 落在 22–24%，约 1900–2100 RPM）；GPU 曲线低频段压得更低、高频段更陡，因为 Max-Q 显卡空闲只有 30–40°C，真打游戏时却会冲到 80°C 以上。

### 四道防抖

借鉴 NBFC-Linux 的 Up/DownThreshold、ClevoFanControl 的平滑调速、asus-wmi 的阶梯滞回：

1. **控制温度只用一个信号**：升温立即跟随，降温按 1/8 衰减（时间常数约 1.6 秒），并且最多只比实测低 4°C。
   早先的做法是「升温看瞬时值、降温看 6 秒均值」，两个信号在升温斜坡上必然相差 3°C 以上，于是每一轮都是「按瞬时值升档、按均值降档」，风扇会在 26%/31% 之间来回跳。改成单一信号后方向只有一种。
2. **占空比死区**：比当前高出 4% 才升、低出 2% 才降。升的阈值必须大于降的单步幅度，否则降完一档立刻又被判成「该升了」，会在同一温度上连写三次 EC（实测 25% → 22% → 24%）。
3. **温度滞回**：降温时控制温度必须低于「当前档位的入口温度 − 3°C」才降档，即 NBFC 的 DownThreshold，按曲线反查入口温度，而不是按占空比拍脑袋留余量。
4. **单步限幅**：每 200ms 一步，升温最多 +20%，降温最多 −3%。突发尖峰不会让风扇瞬间拉满再瞬间落回。

代价是稳定后占空比可能比曲线值低最多 4%（约 2°C 更热），换的是安静与不折腾。

### 临界保护

- 实测温度（不经滤波）≥ 92°C 时无条件 100%，无视曲线与当前档位。
- 锁存到控制温度回落到 86°C（`critical − offset`）以下才交还曲线，避免在 91/92°C 之间反复开关全速。
- **手动档也有安全网**：占空比被设得很低（例如 15%）而温度顶到 93°C 时，自动临时全速；降到 80°C 以下再交还给用户设定的档位。实测 20% 手动档 + 满负载：83°C 时 20%，93°C 触发全速，负载结束后回落到 62°C 时恢复 20%。

### 自定义曲线

曲线读 `~/.config/clevo-indicator/curve`，不存在就用内置默认值：

```
# 温度:占空比，温度必须递增、占空比不许回落，节点 2–10 个
cpu 40:18 48:22 56:28 64:40 72:54 78:68 84:84 89:100
gpu 40:18 55:22 62:30 70:45 76:60 80:75 85:90 88:100
hyst 3              # 降温滞回，°C
critical 92         # 临界温度
critical_offset 6   # 回落多少才解除全速
ramp_up 20          # 单步最大升幅 %
ramp_down 3         # 单步最大降幅 %
mode linear         # linear（线性插值）| step（阶梯跳变）
```

改完重启服务生效，改错了只会影响那一行（保留默认并在日志里提示）。查看解析结果与插值后的完整表格：

```bash
clevo-indicator --curve
```

```
 温度   CPU曲线   GPU曲线   实际取用
 ------------------------------------
  48°C     22%      20%      22%
  50°C     23%      20%      23%
  ...
  92°C    100%     100%     100%  ← 临界全速
```


### 本机实测的占空比—转速对应关系

| 占空比 | 15% | 20% | 25% | 30% | 40% | 50% | 60% |
|---|---|---|---|---|---|---|---|
| 转速 | 1318 | 1730 | 2061 | 2359 | 2933 | 3384 | 3802 |

10% 仍能稳定转动（880 RPM，未失速），但考虑到积灰与轴承磨损会抬高启动门槛，手动下限取 15%。原版 60% 的下限在这台机器上明显过于保守——60% 时 3802 RPM，而空闲只需 20% 就能把温度稳在 48°C。

---

## 安装

### 依赖

```bash
sudo apt-get install libayatana-appindicator3-dev libgtk-3-dev
```

GPU 温度需要 NVIDIA 驱动自带的 `libnvidia-ml.so.1`（`nvidia-utils` / 驱动包一般会装）。它不属于编译依赖——程序用 `dlopen` 在运行时按需加载，缺失时 GPU 温度显示 `--`，其余功能不受影响。

### 编译并安装

```bash
git clone https://github.com/ldbsmh/clevo-indicator.git
cd clevo-indicator
make
make install   # 安装到 /usr/local/bin/，并自动设置 setuid 权限
```

可选：把示例曲线复制成自己的配置（不复制就用内置默认值）：

```bash
mkdir -p ~/.config/clevo-indicator
cp curve.example ~/.config/clevo-indicator/curve
clevo-indicator --curve      # 核对解析结果
```

### 常驻运行（systemd 用户服务）

程序没有退出菜单，由 systemd 用户服务托管，任何原因退出都会在 1 秒后自动拉起：

```bash
mkdir -p ~/.config/systemd/user
cp clevo-indicator.service ~/.config/systemd/user/     # 见下
systemctl --user daemon-reload
systemctl --user enable --now clevo-indicator.service
```

`~/.config/systemd/user/clevo-indicator.service`：

```ini
[Unit]
Description=Clevo Fan Control Indicator
StartLimitIntervalSec=0

[Service]
Type=simple
ExecStart=/usr/local/bin/clevo-indicator
Restart=always
RestartSec=1
KillMode=control-group
TimeoutStopSec=5

[Install]
WantedBy=default.target
```

保活层次（实测结果）：

| 情形 | 结果 |
|---|---|
| UI 主进程崩溃（SIGSEGV） | 1–2 秒内重生，worker 检测到父进程消失后自行退出，不留孤儿 |
| `kill -9` UI 主进程 | 1–2 秒内重生 |
| worker 进程（root） | 普通用户**无权发信号**（setuid 后 real uid = root），只有 root 能杀 |
| 注销 / 重启 | 登录时由 `default.target` 拉起 |

**停止方式**（唯一出口）：`systemctl --user stop clevo-indicator.service`。显式 stop 后 systemd 不会自动重启；`start` 则随时可恢复（启动器条目已经指向 `systemctl --user start`）。

注意 `StartLimitIntervalSec=0` 关掉了 systemd 的启动频率限制，否则短时间内连续崩溃会被判定为失败并放弃重启 —— 这正是「保活」需要的，但也意味着如果配置有误，它可能陷入 1 秒一次的重启循环，此时用 `journalctl --user -u clevo-indicator -n 50` 查原因。


---

## 使用

```bash
# 启动托盘指示器（需要桌面环境）
clevo-indicator

# 查看当前风扇和温度信息
clevo-indicator -?

# 直接设置风扇占空比（15% ~ 100%）
clevo-indicator 80

# 查看自动曲线的解析结果与插值表（只读，服务运行中也能用）
clevo-indicator --curve
```

---

## 注意事项

- 程序使用 setuid=root，但必须由桌面用户启动。内部通过 fork 分成两个进程：子进程以 root 权限操作 EC，父进程以普通用户权限显示托盘图标
- 运行期间请勿使用其他程序通过 `inb/outb` 访问 EC 端口
- 不要用 `kill -9` 强制终止，否则可能导致 EC 命令中断。程序已捕获大多数终止信号以安全退出
- GPU 风扇转速寄存器（`0xD2/0xD3`）在不同 Clevo 机型上可能不同，如读数异常请通过以下命令确认：
  ```bash
  sudo modprobe ec_sys
  sudo od -Ax -t x1 /sys/kernel/debug/ec/ec0/io
  ```

---

## 本机适配记录（Deepin 25 / Terrans Force T1000）

本项目在本机（内核 6.18.56，DDE 桌面）上做了以下必要修改：

| 问题 | 处理 |
|------|------|
| libayatana-appindicator ≥ 0.5 把 `IS_APP_INDICATOR` 改名为 `APP_IS_INDICATOR`，链接报 undefined reference | 加 `#ifndef APP_IS_INDICATOR` 兼容宏，改用 `APP_IS_INDICATOR` |
| 本机 EC `0xCD`（GPU 温度）恒为 0，混合显卡（Optimus）模式下 EC 侧二极管不上报 | 用 `dlopen` 加载 `libnvidia-ml.so.1`，通过 NVML 直接向 NVIDIA 驱动要温度；读不到（无驱动或 GPU 掉电）才显示 `--`，自动调速也只会使用实际可用的温度 |
| 图标主题缺少 `brasero-disc-XX` 动画图标 | 启动时用 `gtk_icon_theme_has_icon` 检测，缺失则回退到静态 `brasero` 图标 |
| GPU 风扇满速实测约 5940 RPM，原 `MAX_FAN_RPM=5500` 会使百分比溢出 | 调整为 6000 |
| `make install` 用 `-m 4750 -g adm`，本机用户不在 `adm` 组导致无法执行 | 改为 `-m 4755` |

### CPU 高温与风扇噪音

本机实测：内核默认 `energy_performance_preference = balance_performance` 时，12 个线程**无论负载高低都被锁在 3.6~3.9GHz 全核睿频**，空闲状态下封装功耗 31W、温度 97~98°C、风扇 100%。

| 电源偏好 | 频率 | 功耗 | 温度 |
|---|---|---|---|
| `balance_performance`（内核默认） | 3900MHz 全核 | 31W | 97~98°C |
| `balance_power` | 3900MHz 全核（**无效**） | — | 98°C |
| `performance` | 3900MHz 全核 | — | 94~97°C |
| `power` | 800MHz 空闲 | ~15W | 50~55°C |

已验证散热器本身正常（把 `max_perf_pct` 压到 30% 后温度 12 秒内 98→61°C），因此这是调频策略问题，不是硬件故障。

DDE 控制中心的「节能模式」不控制 CPU 频率：实测开启后 `energy_performance_preference` 与频率均无变化，`dde-system-daemon` 及 `libdde*`/`libdtk*` 中也没有任何引用 `cpufreq`/`energy_performance_preference` 的代码。因此该档位只能通过本程序的菜单或 `cpu-epp-power.service` 设置。


本机实测数据（CPU 满载 98°C）：

```
CPU FAN Duty: 100%   CPU FAN RPMs: 5097 RPM   GPU FAN RPMs: 5923 RPM
CPU Temp: 98°C        GPU Temp: N/A（EC 未上报）
```

寄存器验证：`0x07` 与 coretemp 读数一致；写入 80% 后 `0xCE` 回读 80%、转速由 5085 降到 4520 RPM，确认写入链路有效。

注意事项：

- **退出程序后风扇保持最后一次手动占空比**，EC 自带的自动调速不会自动接管，直到重启或重新设置
- DDE 会把托盘图标收进折叠区（面板上的 `^`），需要展开才能看到
- 托盘温度文字通过 `XAyatanaLabel` 属性导出，DDE 是否渲染该文字取决于桌面版本；菜单控制不受影响

