from machine import UART, FPIOA
import time


# 按实际开发板修改
TX_PIN = 9
RX_PIN = 10


# 配置引脚复用
fpioa = FPIOA()
fpioa.set_function(TX_PIN, FPIOA.UART1_TXD)
fpioa.set_function(RX_PIN, FPIOA.UART1_RXD)


# 初始化串口
uart = UART(
    UART.UART1,
    baudrate=115200,
    bits=UART.EIGHTBITS,
    parity=UART.PARITY_NONE,
    stop=UART.STOPBITS_ONE,
    timeout=0,
)


END = b"\xff\xff\xff"


def send_command(command):
    """向淘晶驰屏发送一条完整指令。"""
    data = command.encode("utf-8") + END
    uart.write(data)
    time.sleep_ms(10)


# 等待串口屏完成开机和页面加载
time.sleep_ms(1000)


send_command('main.tStatus.txt="通信正常"')
send_command('main.tTarget.txt="正方形"')

# xD/xSize are configured as vvs1=2 on the current screen project.
# Therefore the last two integer digits are displayed after the decimal
# point: 156.30 cm -> 15630, 10.80 cm -> 1080.
send_command("main.xD.val=15630")
send_command("main.xSize.val=1080")
send_command("main.xCurrent.val=823")
send_command("main.xPower.val=412")
send_command("main.xPmax.val=438")

print("screen test commands sent")
