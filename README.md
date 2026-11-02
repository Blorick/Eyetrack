# Eyetrack

嵌入式竞赛 FPGA 赛道工程，包含 PH1P 主板、EG4D 从板，以及运行在 PH1P 内置 RISC-V MCU 上的云台总线舵机程序。

## 1. 首次运行顺序

1. 用 Tang Dynasty（TD）分别构建两块板的 FPGA 位流。
2. 用 FutureDynasty（FD）或其工具链构建 `gimbal_servo.elf`。
3. 将 EG4D 位流下载到 EG4D 板，将 PH1P 位流下载到 PH1P 板。
4. 保持 PH1P 已上电、已配置位流，再连接 MCU 调试服务，烧写 `gimbal_servo.elf` 并运行。

FPGA 的 `.bit`、MCU 的 `.elf` 和硬件描述 `.hpf` 是不同文件：`.bit` 建立硬件逻辑；`.elf` 包含 MCU 程序；`.hpf` 用于让 BSP 与 MCU 硬件配置保持一致。EG4D 不需要烧写本工程的 MCU 程序。

通过 JTAG 下载到 FPGA SRAM 的位流掉电后丢失；MCU 程序按本工程的 `xip_dlm` 链接布局写入 QSPI Flash。若只做 SRAM 配置，重新上电后须先重下 PH1P 位流，MCU 的硬件环境才会恢复。

## 2. 文件与工具

| 内容 | 工程入口或目录 |
| --- | --- |
| PH1P，器件 `PH1P35MDG324-3` | [two_board/PH1P/ph1p_master_core_business.al](two_board/PH1P/ph1p_master_core_business.al) |
| EG4D，器件 `EG4D20EG176` | [two_board/EG4D/eg4d_slave_core_business.al](two_board/EG4D/eg4d_slave_core_business.al) |
| 两板共用 RTL | `two_board/common/` |
| PH1P 引脚、时序约束 | `two_board/PH1P/constraints/` |
| EG4D 引脚、时序约束 | `two_board/EG4D/constr/` |
| MCU Platform/BSP、硬件描述 | `mcu/eyetrack_bsp/`、`mcu/eyetrack_bsp/Eyetrack.hpf` |
| MCU Application | `mcu/gimbal_servo/` |
| 独立仿真配置 | `two_board/sim/*.do` |

构建验证使用 TD **6.2.168116** 和 FD **2026.2**。以下为 Windows PowerShell 命令，按实际安装位置修改变量。需要开多个终端时，在每个终端先执行本段初始化。

```powershell
$ProjectRoot = 'D:\Project\Anlogic\push\Eyetrack'
$TdRoot = 'D:\Program Files\Anlogic\TD_6.2.1_Engineer_6.2.168.116'
$FdRoot = 'D:\Software\FD_Release_2026.2'
$WorkRoot = 'D:\Project\Anlogic\Builds\Eyetrack_programming'
New-Item -ItemType Directory -Path $WorkRoot -Force | Out-Null

$Ocd = "$FdRoot\toolchain\openocd\fpsoc\bin\openocd.exe"
$OcdScripts = "$FdRoot\toolchain\openocd\fpsoc\share\openocd\scripts"
$Gdb = "$FdRoot\toolchain\gcc\riscv\ph1p35\bin\riscv64-unknown-elf-gdb.exe"
```

路径带空格时，保持示例中的 `& "完整可执行文件路径"` 写法。Tcl 文件中的 Windows 路径使用 `/` 或花括号包围。

## 3. 构建 FPGA 位流

推荐使用随工程提供的 `build.tcl`。它保留 DDR IP 约束、PH1P 布局优化以及 EG4D 的打包与 hold 修复设置，并在暂存工程中重定位源码根目录。

### 3.1 PH1P

```powershell
Push-Location "$ProjectRoot\two_board\PH1P"
& "$TdRoot\bin\td_commands_prompt.exe" build.tcl
Pop-Location
```

### 3.2 EG4D

```powershell
Push-Location "$ProjectRoot\two_board\EG4D"
& "$TdRoot\bin\td_commands_prompt.exe" build.tcl
Pop-Location
```

两个构建按顺序运行。确认综合、布局布线、最终时序检查和 bitgen 全部成功；仅有 `.bit` 文件存在不能替代成功检查。

| 构建方式 | PH1P 位流 | EG4D 位流 |
| --- | --- | --- |
| 上述 `build.tcl` | `two_board/PH1P/build/ph1p_master_core_business.bit` | `two_board/EG4D/build/eg4d_slave_core_business.bit` |
| TD GUI 构建 | `two_board/PH1P/ph1p_master_core_business_Runs/best_result/ph1p_master_core_business.bit` | `two_board/EG4D/eg4d_slave_core_business_Runs/best_result/eg4d_slave_core_business.bit` |

使用与本次成功构建对应的文件。TD GUI 打开工程、编译后会产生 `log/`、`*_Runs/`、锁文件等；命令行构建会产生 `build/`。这些是运行生成物，重新整理源码交付包时可排除。

本机曾遇到 TD 自动生成的 `run.bat` 未给带空格的安装路径加引号的问题。若直接运行该 BAT 失败，使用上述 PowerShell 构建入口。

## 4. 构建 MCU 程序

### 4.1 使用 FD 管理工程

在 FD 选择 **File → Open Projects from File System… → Directory**，定位到本工程的 `mcu` 目录，勾选 `eyetrack_bsp` 与 `gimbal_servo` 两个工程，取消父目录本身的勾选，点击 **Finish**。保持工程名称分别为 `eyetrack_bsp` 和 `gimbal_servo`，应用引用的 Platform 为 `eyetrack_bsp`，硬件描述为其目录中的 `Eyetrack.hpf`。

右键 `gimbal_servo` 选择 **Build Project**，或选中应用后使用 **Project → Build Project**。应用 Makefile 会同时从源码构建所需 BSP 库。现有交付版已包含 BSP，无需为了第一次编译重新生成它。

### 4.2 使用 FD 工具链命令行构建

```powershell
$env:PATH = "$FdRoot\toolchain\build-tools\bin;$FdRoot\toolchain\gcc\riscv\ph1p35\bin;$env:PATH"
$CompilePrefix = ($FdRoot.Replace('\', '/') + '/toolchain/gcc/riscv/ph1p35/bin/riscv64-unknown-elf-')

& "$FdRoot\toolchain\build-tools\bin\make.exe" `
  -C "$ProjectRoot\mcu\gimbal_servo" all `
  TARGET=gimbal_servo "COMPILE_PREFIX=$CompilePrefix"
```

默认产物为：

```text
mcu/gimbal_servo/build/gimbal_servo.elf
```

`Makefile` 的 `SDK_ROOT` 默认指向同级 `../eyetrack_bsp`，`DOWNLOAD=xip_dlm` 选用 `lds/gcc_ph1p35_xip_dlm.ld`。烧写时优先使用带地址信息的 ELF。

## 5. FPGA 位流下载

### 5.1 连接与下载模式

给目标板上电，将对应下载线连接电脑。为使命令行中的 `-cable 0` 对应正确板卡，使用下述 PH1P 命令时只保留一条待用的 JTAG 下载线。两板位流必须与器件对应。

已下载另一块板时，必须保持其供电。如果下载 USB 同时也是该板唯一电源，拔线就会丢失 SRAM 配置；应采用板卡支持的独立供电，或按下载工具说明明确选择下载线后保留两板连接。多条下载线同时可见时，不要直接沿用本节的 `-cable 0` 示例。

JTAG/SRAM 下载用于当次上电运行；需要断电后自动启动时，再按第 7 节进行配置 Flash 固化。

### 5.2 PH1P：OpenOCD 桥接 TD 下载器

本机 PH1P 板载 FT2232H 的 OpenOCD/D2XX 路径已有成功记录。下面使用 FD 安装内的工具建立 remote-bitbang 桥，再由 TD 下载器写入位流。

先断开其它占用该下载线的 TD 下载窗口、硬件服务或调试会话。桥接服务和 MCU 调试服务分阶段启动，不要同时占用下载线。

**终端 A：启动 FPGA 配置桥。**

```powershell
& $Ocd -s $OcdScripts `
  -c 'tcl_port 6666; telnet_port 4444' `
  -f target/anlogic/onecable.cfg `
  -c 'init; rbb server start 5555'
```

等待日志出现监听 `5555` 的 rbb 服务。此阶段只使用 JTAG 适配器，不预先声明 RISC-V TAP；因此也适用于 PH1P 尚未配置位流的状态。

**终端 B：生成下载命令并执行。**

```powershell
$Ph1pBit = (Join-Path $ProjectRoot 'two_board\PH1P\build\ph1p_master_core_business.bit').Replace('\', '/')
$DownloadLines = @(
  'connect_cable -host_name 127.0.0.1:5555'
  'set rc [catch {'
  ('  download -bit {' + $Ph1pBit + '} -mode jtag_burst -spd 7 -cable 0')
  '} message]'
  'puts "DOWNLOAD rc=$rc : $message"'
  'if {$rc} {exit 1}'
  'exit 0'
)
[IO.File]::WriteAllText(
  (Join-Path $WorkRoot 'download_ph1p.tcl'),
  ($DownloadLines -join "`n"),
  [Text.UTF8Encoding]::new($false)
)

Push-Location $WorkRoot
& "$FdRoot\tools\td\bw_commands_prompt.exe" download_ph1p.tcl
Pop-Location
```

若使用 GUI 构建的位流，先将 `$Ph1pBit` 改为第 3 节表中的 GUI 产物路径。成功应同时看到器件校验通过、下载进度完成以及 `DOWNLOAD rc=0`。

**终端 B：下载完成后，正常关闭本节启动的 OpenOCD 桥。**

```powershell
$OcdClient = [Net.Sockets.TcpClient]::new('127.0.0.1', 4444)
$OcdStream = $OcdClient.GetStream()
$ShutdownBytes = [Text.Encoding]::ASCII.GetBytes("shutdown`r`n")
$OcdStream.Write($ShutdownBytes, 0, $ShutdownBytes.Length)
$OcdClient.Dispose()
```

等待终端 A 的 OpenOCD 退出，保持 PH1P 供电，然后进入第 6 节。优先使用 `shutdown` 正常结束；强制终止 OpenOCD 曾导致本机 FT2232 通道异常、需要重新插拔。

### 5.3 TD GUI 下载：两板通用，EG4D 按此操作

1. 在 TD 选择 **Tools → BitWriter**，或在 FPGA Flow 面板双击 **Bitwriter**。
2. 连接目标板的下载线，点击 **Refresh**，核对器件：PH1P 应为 `PH1P35MDG324`，EG4D 应为 `EG4D20EG176`。
3. 点击 **Add**，选择第 3 节表中与目标板对应的本次 `.bit` 文件。
4. 将 **Mode** 设为 **PROGRAM_SRAM**，选择与下载线、目标板相适应的速度，点击 **Run**。
5. 等待进度完成并确认工具报告成功。此时位流已进入 FPGA SRAM，掉电后需要重新下载。

PH1P 的 TD 直连下载要求下载线驱动与 TD 硬件服务匹配；如果本机保持 FTDIBUS/D2XX 驱动、TD 直连无法打开下载线，可使用第 5.2 节的命令行桥接方法，或按第 5.4 节让 GUI 连接该桥。EG4D 的下载线与驱动按其板卡和工具实际枚举结果选择。

下载完成后保持板卡供电，再切换到下一块板或 MCU 调试。下载和 MCU 调试之间要释放下载线连接。**BitWriter** 是此处使用的下载窗口；手册标题为 *BitWizard 使用指南*。不要把 ChipWatcher 调试窗口当作位流下载入口。

### 5.4 让 TD GUI 连接 OpenOCD 桥

对于第 5.2 节使用的 PH1P 下载线，可按 OneCable 官方文档配置远程调试服务器，让 BitWriter 通过 RBB 桥访问下载线：

1. 按第 5.2 节“终端 A”启动桥，确认 RBB 监听 `5555`。
2. TD 中选择 **Tools → Debug Server Setting**，选择 **AL-LINK-FT(Remote)**。
3. 用右侧绿色 **+** 添加服务器，IPv4 地址填 **127.0.0.1**，端口填 **5555**，点击 **Apply and Close**。
4. 打开 BitWriter，按第 5.3 节执行 **PROGRAM_SRAM**，或按第 7 节执行 **PROGRAM_FLASH**。
5. GUI 操作结束后释放下载连接，再用第 5.2 节的 `shutdown` 方法关闭桥；切换到 MCU 调试服务时保持 PH1P 供电。

这里连接的是已经启动的 OpenOCD RBB 服务，不要再让 Local Host 的另一硬件服务占用同一下载线/端口。OneCable 手册给出了 Remote/RBB 机制；本机已有命令行桥接成功记录，但这组 GUI 远程设置与 Flash 固化尚未在精简交付版上实测。

## 6. 烧写并运行 PH1P MCU 程序

### 6.1 本工程的关键配置

| 项目 | 当前值 |
| --- | --- |
| 应用 ELF | `mcu/gimbal_servo/build/gimbal_servo.elf` |
| 链接模式 | `xip_dlm`，代码位于 QSPI Flash，数据位于 DLM |
| MCU 复位向量 / Flash 链接起点 | `0x20E00000` |
| Flash 窗口基址 / MCU 镜像偏移 | `0x20000000` / `0xE00000`（14 MiB） |
| ELF 入口 `_start` | `0x20E00100` |
| DLM | `0x08008000`，16 KiB |
| MCU 核心 / Timer / AHB 时钟 | 300 / 100 / 150 MHz |
| JTAG | `MCU_JTAG_SEL=0`，使用主 FPGA JTAG 链 |
| 舵机输出 | UART1，115200、8N1、只发送；`O_servo_bus` 对应 FPGA 引脚 C18 |

**前置条件：PH1P 已配置本工程位流，且第 5 节的桥接 OpenOCD 已退出。** FPGA 未配置时，RISC-V 调试目标还不可用，直接启动 MCU 下载会出现 `Target not examined`、`dtmcontrol is 0` 等错误。

### 6.2 终端 A：启动 MCU OpenOCD 服务

```powershell
& $Ocd -s $OcdScripts `
  -c 'gdb_port 3333; telnet_port 4444; tcl_port 6666' `
  -f target/anlogic/ph1p35.cfg
```

官方 `ph1p35.cfg` 默认使用 `CONFIG_JTAG=0`，声明 RISC-V CPU 与 FPGA 两个 TAP，配置 QSPI Flash 下载算法并初始化目标。等待出现目标检查成功以及监听 `3333` 的 GDB 服务。

不要使用 FD 的 `Program FPGA` 选项代替上述先下位流的步骤：本机曾出现先检查 RISC-V、后准备下位流的顺序死锁。此处已单独完成 FPGA 下载，不需要让 FD 再配置一次 FPGA。

### 6.3 终端 B：使用 GDB 烧写 ELF

```powershell
$ElfFile = Join-Path $ProjectRoot 'mcu\gimbal_servo\build\gimbal_servo.elf'
& $Gdb -q $ElfFile
```

在 GDB 提示符中逐行输入：

```gdb
set confirm off
set pagination off
set mem inaccessible-by-default off
set remotetimeout 250
set arch riscv:rv32
target extended-remote 127.0.0.1:3333
monitor halt
load
monitor reset halt
p/x $pc
continue
```

`load` 根据 ELF 的段地址执行下载；配合本节的 OpenOCD Flash bank，会写入 QSPI Flash，而非只把程序放进临时 RAM。成功记录应包含从 `0x20e00000` 开始的段下载、`Start address 0x20e00100`；本工程在 `reset halt` 后的 PC 预期为 `0x20e00100`。这条流程无需再叠加一次 `flash write_image erase`。

本机历史记录识别的 Flash 为 16 MiB 器件，因此从 14 MiB 偏移起实际还剩 2 MiB；当前需写入 Flash 的数据约 32 KiB，位于该范围内，包含调试信息的 ELF 文件本身可以更大。链接脚本中的 `LENGTH=4M` 是原工程声明，不代表这颗 Flash 从该偏移起有 4 MiB 物理容量。后续若扩大固件，需按实际器件容量和分区重新检查。

`continue` 后程序持续运行、GDB 不立即返回提示符是正常现象。首次运行会发送两轴居中指令；随后每约 20 ms 读取 FPGA 的 AHB 云台角度寄存器并发送舵机总线帧。连接好舵机电源、共地及数据线，并给云台留出活动空间。

当前程序默认不通过 UART0 打印日志，板载 USB 串口没有文字输出不能单独判定烧写失败。可通过 GDB 的入口/`main` 执行情况、C18 上的 UART1 波形以及舵机动作检查运行状态。

需要结束调试时，在 GDB 中按 `Ctrl+C` 返回提示符，再执行 `detach`、`quit`，随后使用第 5.2 节的 `shutdown` 方法正常结束该 OpenOCD 服务。

## 7. 掉电启动与配置 Flash 固化

SRAM 下载成功只代表本次上电已配置 FPGA。若要演示时断电重启即可运行，需要分别将两块板的 FPGA 位流固化到其配置 Flash，并确认板卡启动模式；MCU ELF 仍需按第 6 节单独烧写。

### 7.1 用 BitWriter 固化两板位流

对 PH1P、EG4D 分别执行：

1. 先按第 5.3 节建立 TD 下载连接；PH1P 若使用 OpenOCD 桥，则按第 5.4 节设置远程服务。打开 **Tools → BitWriter**，点击 **Refresh** 并核对目标器件。
2. **Add** 对应板的 `.bit` 文件，将 **Mode** 改为 **PROGRAM_FLASH**。
3. 按实际板卡配置 Flash 的容量设置 **MemorySize**，选择适当的 **Speed**。手册截图中的 `256Mbits` 是示例，不能直接套用。128 Mbit 等于 16 MiB；本机 PH1P 历史记录识别为 `GD25Q128C`，具体型号和容量仍以实际板卡及工具识别为准。
4. 点击 **Run**，等待编程完成；使用 **Verify Flash** 核验写入数据，确认成功后再断开下载连接。
5. 按对应开发板手册将启动模式设为从配置 Flash 启动。两块板的启动跳线/拨码位置应以各自板卡版本为准，此处不指定未经核实的拨码编号。
6. PH1P 固化并设置好启动模式后，先重新上电从 Flash 配置，或按第 5 节再下载一次同版 SRAM 位流，恢复 MCU 的硬件环境；再按第 6 节烧写 MCU ELF。最后对两板断电重启，检查它们是否自动配置，以及云台 MCU 是否正常运行。

PH1P 工程保留了 `boot_mode=mspix4`、`mclk_freq=33MHz`：它们分别表示 **MSPI x4 上电启动**和**读取配置 Flash 的时钟**，不是下载器的 JTAG 模式或 TCK 频率。不要为了调整下载速度随意修改它们。

### 7.2 PH1P 的 FPGA 与 MCU 共用配置 Flash

PH1P 数据手册图 2-21 显示，硬核 MCU 的 **QSPI0** 与 FPGA **CONFIG** 经复用连接同一颗配置 Flash。MCU ELF 的 `0x20E00000` 是 CPU 地址，换算为该 Flash 内偏移是 `0xE00000`。不要将此偏移用作 FPGA 主位流的烧写位置，也不要将 ELF 作为 BitWriter 的位流文件。

**BitWriter 的 Erase Flash 是整片擦除，会同时删除 PH1P 上的 MCU 程序。** 若需要执行，应安排在固化 FPGA 之前，随后重新烧写 MCU ELF。推荐顺序始终为：

```text
固化 EG4D FPGA → 固化 PH1P FPGA → 烧写 PH1P MCU ELF → 两板断电重启验收
```

重新固化/擦除 PH1P 配置 Flash 后，要重新确认 MCU 固件是否仍在，必要时重烧 ELF。板上另接用户 IO 的 SPI Flash 与上述配置 Flash 不能混为一谈。

本节 Flash 固化步骤依据官方文档整理，当前精简交付版尚未完成实际固化和双板断电自启动实测。

## 8. 修改硬件、迁移目录

- 两个 `.al` 的 `Project Path`、两份 `mcu/*/.fd/project.json` 的 `location`、`bspLocation`、`hpfFile` 使用当前交付位置的绝对路径。移动目录后应更新这些字段；两个 `build.tcl` 会按自身位置重定位暂存 AL，MCU Makefile 的 BSP 路径则使用相对路径。
- 只改 MCU 应用 C/H：重新编译 ELF，再执行第 6 节；同次上电且 FPGA 未改动时不必重下位流。
- 修改 MCU IP 时钟、外设使能、复位向量或 JTAG 配置：重新构建 PH1P、导出对应 HPF、在 FD 更新 Platform/BSP、重新编译并下载 ELF。
- 只改 FPGA 引脚约束：重新构建并下载对应位流；MCU IP 配置未变时无需因此重生成 BSP。
- 交付中的 `Eyetrack.hpf` 与当前 MCU IP 配置相匹配。SDK 通用的独立 `make hpf` 辅助目标不属于本工程支持的构建入口。

## 9. 常见问题

| 现象 | 检查与处理 |
| --- | --- |
| 找不到下载线、无法占用 USB | 核对目标板供电、数据线及驱动；释放其它占用下载线的下载/调试会话。 |
| PH1P 的 OpenOCD 提示尝试 D2XX | 本机 FTDIBUS 驱动下存在自动回退 D2XX 的成功记录；只要后续连接、rbb/目标检查成功，不必仅因该提示改驱动。 |
| USB 只出现复合父设备，没有 JTAG/串口子接口 | 本机曾因将 WinUSB 绑定到整个 FT2232 复合父设备而发生此故障。父设备应由 Windows 复合设备驱动接管；勿把父设备与 JTAG 的 MI_00 子接口混为一谈。 |
| `Target not examined`、`dtmcontrol is 0` | 先确认 PH1P 已下载本工程位流；重新上电后若未固化配置 Flash，先重下位流。 |
| OpenOCD 端口占用 | 桥接阶段与 MCU 调试阶段复用 4444/6666，先正常退出前一阶段服务。 |
| GDB 下载到错误地址或 reset 后跑飞 | 核对 ELF 来自当前 `xip_dlm` 构建，Flash 起点与 MCU reset vector 均为 `0x20E00000`。 |
| 烧写后串口没有日志 | 当前应用使用 UART1 发舵机指令，UART0 未启用；按第 6.3 节检查。 |
| 舵机无动作 | 核对 C18 数据线、供电/共地、舵机 ID（Pan=0、Tilt=1）和 115200 波特率；检查 MCU 是否已运行及 AHB 基址 `0x80010000`。 |
| 改 TD 硬件后 MCU 行为异常 | 核对新位流、HPF、BSP、链接脚本与 ELF 是否来自同一配置。 |

## 10. 验证范围与参考资料

交付整理时，两板综合、布局布线、最终 STA、位流生成及 MCU 从源码编译均已通过。PH1P setup/hold WNS 为 0.055/0.019 ns，EG4D 为 0.004/0.004 ns；该数值对应整理时的一次构建，不保证每次实现结果逐字节一致。44 个独立 testbench 中，41 项完整运行通过，3 项长集成完成编译、展开及有限启动检查。

本 README 中的 PH1P 桥接下载和 MCU `load → reset halt → continue` 方法参考了同板卡、同工具链的历史成功记录；精简后的 Eyetrack 尚未重新进行实际烧写、完整上板或断电自启动验收。

可查阅随工具安装提供的原厂文档：

- `<TD>/doc/SWUG/SWUG110_BitWizard_User_Guide.pdf`：印刷第 10–12 页为 BitWriter 入口及 Refresh/Add/Run；第 18–22 页为 PROGRAM_SRAM、PROGRAM_FLASH、Verify Flash 与整片擦除。
- `<TD>/doc/Tools_UG/UG009_OneCable_User_Guide.pdf`：印刷第 5、7–8 页为 Debug Server Setting 与 AL-LINK-FT(Remote)；第 10 页说明 TD 使用 OpenOCD RBB 服务进行 PL 位流下载。
- `<TD>/doc/TD_User_Guide_V6.2.pdf`：第 7 章、印刷第 164 页为 Flow 面板 Bitwriter 入口；附录说明位流启动配置。
- `<FD>/doc/SWUG500_FD_User_Guide.pdf`：§2.1、印刷第 7–9 页为现有工程导入；§5.3、第 61–62 页为构建；§5.4、第 63–69 页为调试配置。§5.7 的 XIP/BOOT.bin 教程使用 DR1V90 示例，不作为本工程 PH1P35 的烧写步骤。
- `DS1300_PH1P_Datasheet`：印刷第 43 页／PDF 第 49 页，图 2-21，说明 QSPI0 与 CONFIG 共用配置 Flash。
- FD 安装内 `toolchain/openocd/fpsoc/share/openocd/scripts/target/anlogic/onecable.cfg`、`ph1p35.cfg`、`init.cfg`：本 README 使用的下载线及 MCU 调试配置。

工程仿真须按配置注释使用 `source`，例如在独立的可写输出目录中执行：

```text
vsim -c -do "source {D:/Project/Anlogic/push/Eyetrack/two_board/sim/tb_fill_policy.do}"
```

仿真库、日志、波形及数据副本写入启动时的当前目录。直接将这些配置作为 Questa `do` 宏执行，会影响其源码根目录定位。
