# 淘晶驰电阻触摸屏端配置

本文件对应 `tjc_screen.py` 的固定协议。屏幕工程需要在淘晶驰编辑器中配置一次；K230 端只发送 ASCII 数值控件命令，不发送中文文本。

## 1. 全局变量、控件和定时器

在 `program.s` 中加入：

```c
int workMode=0
int targetId=0
int lastStatus=255
bkcmd=0
bauds=115200
page main
```

`main` 页需要包含以下全局控件：`nStatus`（隐藏数字状态码）、`xD`、`xSize`、`xCurrent`、`xPower`、`xPmax`（虚拟浮点数）、`tStatus` 和 `tTarget`（文本）。另加：

```text
tmStatus: tim=100, en=1
tmIdle:   tim=3000, en=0
```

## 2. 模式按钮

模式按钮只修改屏幕内部变量和目标文字，不直接触发视觉测量：

```c
// 自动
workMode=0
targetId=0
tTarget.txt="自动识别"
tStatus.txt="待机"
nStatus.val=0

// 最小正方形
workMode=1
targetId=0
tTarget.txt="最小正方形"
tStatus.txt="待机"
nStatus.val=0

// 编号 N（把 N 替换为 0～9）
workMode=2
targetId=N
tTarget.txt="编号N"
tStatus.txt="待机"
nStatus.val=0

// 倾斜 30°～60°
workMode=3
targetId=0
tTarget.txt="倾斜正方形"
tStatus.txt="待机"
nStatus.val=0
```

## 3. 测量按钮弹起事件

测量按钮的 `Touch Release Event`：

```c
if(nStatus.val!=1)
{
  nStatus.val=1
  tStatus.txt="测量中"
  printh 55 01
  prints workMode,1
  prints targetId,1
  printh ff ff ff
}
```

发送帧严格为 7 字节：`55 01 MODE ID FF FF FF`。 `MODE=0` 基本题、`1` 最小正方形、`2` 指定编号、`3` 倾斜正方形。

## 4. 状态显示和自动回待机

`tmStatus` 的定时事件：

```c
if(nStatus.val!=lastStatus)
{
  lastStatus=nStatus.val
  tmIdle.en=0
  if(nStatus.val==0)
  {
    tStatus.txt="待机"
  }
  else if(nStatus.val==1)
  {
    tStatus.txt="测量中"
  }
  else if(nStatus.val==2)
  {
    tStatus.txt="测量成功"
    tmIdle.en=1
  }
  else if(nStatus.val==3)
  {
    tStatus.txt="测量失败"
    tmIdle.en=1
  }
  else if(nStatus.val==4)
  {
    tStatus.txt="未检测到靶纸"
    tmIdle.en=1
  }
  else if(nStatus.val==5)
  {
    tStatus.txt="图形识别失败"
    tmIdle.en=1
  }
  else if(nStatus.val==6)
  {
    tStatus.txt="编号未找到"
    tmIdle.en=1
  }
  else
  {
    tStatus.txt="系统异常"
    tmIdle.en=1
  }
}
```

`tmIdle` 的定时事件：

```c
tmIdle.en=0
nStatus.val=0
lastStatus=0
tStatus.txt="待机"
```

屏幕启动完成后，在 `main` 页初始化事件发送握手：

```c
printh 55 03 00 00 ff ff ff
```

## 5. K230 端数据和接线

K230 通过 `GPIO9/排针3=UART1_TXD`、`GPIO10/排针4=UART1_RXD`，以 `115200 8N1` 发送：

```text
main.nStatus.val=<0..7>
main.xD.val=round(D_cm*100)
main.xSize.val=round(x_cm*100)
main.xCurrent.val=round(I_A*1000)
main.xPower.val=round(P_W*100)
main.xPmax.val=round(Pmax_W*100)
```

每条 ASCII 指令末尾均为 `FF FF FF`；K230 不写 `tStatus` 和 `tTarget`。

```text
K230 排针3 GPIO9/UART1_TXD  -> 屏幕 RX
K230 排针4 GPIO10/UART1_RXD <- 屏幕 TX
K230 GND                    -- 屏幕 GND
```

两端共地、TX/RX 交叉，确认屏幕 TX 不超过 K230 RX 的电平。USB-C 可以同时连接 CanMV IDE；不要把 USB-TTL 的 VCC 接到 K230。

## 6. 联调顺序

1. 先将 `tools/k230/LCD_test.py` 临时复制到 K230 的 `/sdcard` 根目录并运行，确认五个数字控件能更新。
2. 烧录本文件中的屏幕事件，确认 IDE 中出现 `SCREEN_RX`。
3. 点击自动、最小、编号和倾斜按钮，分别检查 `task=AUTO_MIN`、`DIGIT_SELECT`、`TILT_MIN`。
4. 接入功率采样后发送 `55 02 00 00 FF FF FF`，确认 `Pmax` 清零。
