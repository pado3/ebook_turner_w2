'''
eBook_turner_w2.py
e-book page turner for android with BLE ver.2 by @pado3
target device: Seeed studio XIAO nRF52840 Sense (w/LSM6DS3TR-C)

r1.0 2023/02/15 initial release
r1.1 2023/02/19 minor modification. append, translate and correct comments
r1.2 2023/02/19 fix typo
r1.3 2023/02/20 fix typo

memo.
Reader & Kindle : forward page with volume decrement, reverse with increment
Kinoppy : forward page with volume increment, reverse with decrement
読書尚友 & なろうリーダ : selectable
FWD:D3+4, REV:D8+9, BACK/POWER:D5+6, mode:D7 (3,8,5,7:input, 4,9,6:interrupt)
mouse click almost center of screen : INT1 with double tap
D3, D8:internal pullup (typ.13k)
D5:external pullup 100k(use interrupt with deep sleep)
D7:external pullup 100k(internal is too small when Kinoppy keep low)
external LED Anode:D1(always True), Kathode:D0 (reverse logic same as internal)
'''
import alarm
import analogio     # use .AnalogIn() only
import board
import digitalio
import microcontroller
import supervisor   # use .reload() only
import time
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.standard import BatteryService
from adafruit_ble.services.standard.hid import HIDService
from adafruit_hid.consumer_control import ConsumerControl
# libraries related to sensor
import busio        # use .I2C() only
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_hid.mouse import Mouse
from adafruit_lsm6ds.lsm6ds3trc import LSM6DS3TRC   # use CHIP_ID only
from adafruit_register.i2c_bit import RWBit
from adafruit_register.i2c_bits import RWBits
from adafruit_register.i2c_bit import ROBit
from micropython import const


# LED logic of XIAO is reversed and often confusing, so make clarify
LED_ON = False
LED_OFF = True


# IMU interrupt configuration and readout registers
class ImuInt1Control:
    def __init__(self, i2c):
        self.i2c_device = i2c  # self.i2c_device required by RWBit class
    # see ST_LSM6DS3TR-C datasheet
    # R/W resisters. #:used in this project
    INT1_CTRL = RWBits(8, const(0x0D), 0)
    CTRL1_XL = RWBits(8, const(0x10), 0)    # ODR_XL is upper 4bit
    CTRL2_G = RWBits(8, const(0x11), 0)
    CTRL3_C = RWBits(8, const(0x12), 0)     # use 1bit as SW_RESET
    CTRL4_C = RWBits(8, const(0x13), 0)
    CTRL5_C = RWBits(8, const(0x14), 0)
    CTRL6_C = RWBits(8, const(0x15), 0)     # use 1bit as XL_HM_MODE
    CTRL7_C = RWBits(8, const(0x16), 0)     # use 1bit as G_HM_MODE
    CTRL8_XL = RWBits(8, const(0x17), 0)
    CTRL9_XL = RWBits(8, const(0x18), 0)
    CTRL10_C = RWBits(8, const(0x19), 0)
    MASTER_CONFIG = RWBits(8, const(0x1A), 0)
    TAP_CFG = RWBits(8, const(0x58), 0)     #
    TAP_THS_6D = RWBits(8, const(0x59), 0)  #
    INT_DUR2 = RWBits(8, const(0x5A), 0)    #
    WAKE_UP_THS = RWBits(8, const(0x5B), 0)     #
    WAKE_UP_DUR = RWBits(8, const(0x5C), 0)
    FREE_FALL = RWBits(8, const(0x5D), 0)
    MD1_CFG = RWBits(8, const(0x5E), 0)     #
    MD2_CFG = RWBits(8, const(0x5F), 0)
    ODR_XL = RWBits(4, const(0x10), 4)      # Accelerometer Output Data Rate
    SW_RESET = RWBit(const(0x12), 0)        # CTRL3_C bit0
    XL_HM_MODE = RWBit(const(0x15), 4)      # CTRL6_C bit4
    G_HM_MODE = RWBit(const(0x16), 7)       # CTRL7_C bit7
    # Read only registers in bit. #:used in this project
    # WAKE_UP_SRC
    FF_IA = ROBit(const(0x1B), 5)
    SLEEP_STATE_IA = ROBit(const(0x1B), 4)
    WU_IA = ROBit(const(0x1B), 3)
    X_WU = ROBit(const(0x1B), 2)
    Y_WU = ROBit(const(0x1B), 1)
    Z_WU = ROBit(const(0x1B), 0)
    # TAP_SRC
    TAP_IA = ROBit(const(0x1C), 6)
    SINGLE_TAP = ROBit(const(0x1C), 5)
    DOUBLE_TAP = ROBit(const(0x1C), 4)      # W tap detection
    TAP_SIGN = ROBit(const(0x1C), 3)
    X_TAP = ROBit(const(0x1C), 2)
    Y_TAP = ROBit(const(0x1C), 1)
    Z_TAP = ROBit(const(0x1C), 0)
    # D6D_SRC
    DEN_DRDY = ROBit(const(0x1D), 7)
    D6D_IA = ROBit(const(0x1D), 6)
    ZH = ROBit(const(0x1D), 5)
    ZL = ROBit(const(0x1D), 4)
    YH = ROBit(const(0x1D), 3)
    YL = ROBit(const(0x1D), 2)
    ZH = ROBit(const(0x1D), 1)
    ZL = ROBit(const(0x1D), 0)


# protect VBATT from over voltage in battery operation (>3.6V)
# cannot set low during deep sleep. Do not fall into deep sleep while charging.
def vbatt_port_guard():
    # set P0.14 to LOW
    ebat = digitalio.DigitalInOut(board.READ_BATT_ENABLE)
    ebat.direction = digitalio.Direction.OUTPUT
    ebat.value = False  # MUST be set low in battery operatoin
    time.sleep(0.1)     # wait for ebat pin to GND


# set Li-Po charge mode. HIGH should use for over 500mAh (0.2C=100mA)
# 'HIGH':100mA, other:50mA
def battery_charge_mode(mode='LOW'):
    # prepare to be called multiple times
    try:
        chgl = digitalio.DigitalInOut(microcontroller.pin.P0_13)
    except Exception:
        chgl.deinit()
        chgl = digitalio.DigitalInOut(microcontroller.pin.P0_13)
    if mode == 'HIGH':
        print('charge mode is HIGH (100mA). do not use for < 500mAh battery')
        print('note: this mode will cancel in DEEP sleep')
        chgl.direction = digitalio.Direction.OUTPUT
        chgl.value = False
    else:
        print('charge mode is LOW (50mA).')
        chgl.direction = digitalio.Direction.INPUT  # set to Hi-Z explicitly


# LED settings
def define_led():
    led_array = []  # 0:RED, 1:GREEN, 2:BLUE, 3:additional
    led_pins = [
        board.LED_RED,      # 2.2k
        board.LED_GREEN,    # 10k (?)
        board.LED_BLUE,     # 2.2k
        board.D0,           # additional LED with low current (10k, 0.1mA)
    ]
    for pin in led_pins:
        led_pin = digitalio.DigitalInOut(pin)
        led_pin.direction = digitalio.Direction.OUTPUT
        led_pin.value = LED_OFF
        led_array.append(led_pin)
    # power for anode of additional LED
    aled_a = digitalio.DigitalInOut(board.D1)
    aled_a.direction = digitalio.Direction.OUTPUT
    aled_a.value = True
    return led_array


# Switch settings
def define_switch():
    sw_array = []   # 0:Forward, 1:Reverse, 2:BACK/POWER, 3:mode
    sw_pins = [board.D3, board.D8, board.D5, board.D7]
    for pin in sw_pins[:2]:     # internal pullup for D3, D8
        sw_pin = digitalio.DigitalInOut(pin)
        sw_pin.direction = digitalio.Direction.INPUT
        sw_pin.pull = digitalio.Pull.UP     # note: 13kohm
        sw_array.append(sw_pin)
    for pin in sw_pins[2:]:     # external pullup for D5, D7
        sw_pin = digitalio.DigitalInOut(pin)
        sw_pin.direction = digitalio.Direction.INPUT
        # sw_pin.pull = digitalio.Pull.UP   # external 100kohm
        sw_array.append(sw_pin)
    return sw_array


# define and initialize sensor as W-tap detector, return interrupt object
def define_sensor():
    # define IMU power and turn on
    imupwr = digitalio.DigitalInOut(board.IMU_PWR)
    imupwr.direction = digitalio.Direction.OUTPUT
    imupwr.value = True
    time.sleep(0.1)
    imu_i2c = busio.I2C(board.IMU_SCL, board.IMU_SDA)
    imu = LSM6DS3TRC(imu_i2c)   # use for get I2C address(0x6A) only
    imu_device = I2CDevice(imu_i2c, imu.CHIP_ID)   # CHIP_ID: I2C address
    int1c = ImuInt1Control(imu_device)
    # reset IMU and set low/normal power mode
    int1c.SW_RESET = 1
    int1c.XL_HM_MODE = 1
    int1c.G_HM_MODE = 1
    # INT1 settings. (*):tuning factor, others:fixed parameter
    # upper 4bit of 10h=ODR_XL is important for response & current.
    # ODR: Output Data Rate. I set 208Hz (~5ms), max freq in normal power mode
    int1c.CTRL1_XL = 0x58       # 10h, 0101 1000 Power-up, 208Hz(*), FS+-4g
    int1c.TAP_CFG = 0x8E        # 58h, 1000 1110 INT_EN, keep ODR, XYZ
    int1c.TAP_THS_6D = 0x0A     # 59h, 0000 1010 ths:10/32(=1.25g)(*)
    # INT_DUR2, ODR_XL time is 1/f at CTRL1_XL. GAP~.5s, Q~.04s, DURmax~.08s
    int1c.INT_DUR2 = 0x3A       # 5Ah, 0011 1010 duration, quiet setting (all*)
    int1c.WAKE_UP_THS = 0x88    # 5Bh, 1000 1000 S&W tap EN, wakeup ths:8/64(*)
    int1c.MD1_CFG = 0x08        # 5Eh, 0010 1001 routing W tap only
    '''
    # sample parameters by chuck '22/5 on Seeed forum (same as AN5130 of STM)
    # title: 'XIAO BLE Sense - LSM6DS3 INT1 Single Tap Interrupt'
    int1c.CTRL1_XL = 0x60       # 10h, 0110 0000 416Hz(~2.5ms), FS_XL+-2g
    int1c.TAP_CFG = 0x8E        # 58h, 1000 1110 INT_EN, SLOPE_FDS, XYZ
    int1c.TAP_THS_6D = 0x8C     # 59h, 1000 1100 6D, ths:12/32(=0.75g) (why 6D?)
    int1c.INT_DUR2 = 0x7F       # 5Ah, 0111 1111 GAP~.6s, Q~.03, DURmax~.06s
    int1c.WAKE_UP_THS = 0x80    # 5Bh, 1000 0000 S&W EN, ths:none
    int1c.MD1_CFG = 0x08        # 5Eh, 0000 1000 W to INT1
    '''
    return int1c


# calculate battery level in percent and set low battery alart
def battery_percent(readout, led_array):
    # vbat[mV] = raw*3300[mV]*((1M+510k)/510k)/2^16 in linear region (Vdd>3.3V)
    vbat = readout * (3300 / 65536) * (1510 / 510)
    # experimentally, 100% : 3.7V~23900raw, 0% : 2.5V~21400raw
    # see: https://twitter.com/pado3/status/1613699618092744704/photo/3
    pc = int(100 * (readout - 21400) / (23900 - 21400))
    print('VBATT:{:.0f}mV, {}%, '.format(vbat, pc), end='')
    # percentage limitation in BatteryService : uint8, 0~100
    if pc > 100:
        pc = 100
    elif pc < 0:
        pc = 0
    # low battery alart
    if pc < 20:     # below 20% (~3.3V), RED LED always ON
        led_array[0].value = LED_ON
    else:
        led_array[0].value = LED_OFF
    return pc


# ble disconnection
def ble_disconnection(ble):
    if ble.connected:
        for connection in ble.connections:
            connection.disconnect()
        print('disconnect. ', end='')


# illumination until advertising
def ble_wait_connection(i, led_array):
    if i % 100 == 0:
        led_array[1].value = LED_ON
    if i % 10 == 0:
        led_array[2].value = LED_ON
        print('{}'.format(60 - int(i/10)), end='')
    else:
        print('.', end='')
    time.sleep(0.02)    # blink short
    led_array[1].value = LED_OFF
    led_array[2].value = LED_OFF
    time.sleep(0.08)


# ble advertisement
def ble_advertisement(ble, advertisement, tadv, sw_array, led_array):
    ble.start_advertising(advertisement)
    i = 0
    while not ble.connected and i <= tadv*10:   # wait for connection tadv[s]+a
        ble_wait_connection(i, led_array)   # spend 0.1 sec
        i += 1
        if not sw_array[2].value:   # BACK/POWER is pressed
            break
    ble.stop_advertising()


# check W tap status and return its keycode
def check_sensor(int1c, keycode=0x00):
    wt = int1c.DOUBLE_TAP
    print('W_TAP: {}, '.format(wt), end='')
    if wt:
        keycode = 0x76  # instead of 'Keyboard Menu' in USB HID Usage Tables
    return keycode


# check key status and return its keycode
def check_switch(sw_array, keycode=0x00):
    # standard: https://www.usb.org/sites/default/files/hut1_21_0.pdf
    # default order: Reader/Kindle mode
    keycodes = [
        0xEA,   # Volume Decrement p.120, FWD in Reader/Kindle mode
        0xE9,   # Volume Increment p.120, FWD in Kinoppy mode
        0x224,  # AC BACK p.124
        0x30,   # Power p.118
    ]
    if not sw_array[3].value:       # Kinoppy mode, swap FWD & REV
        keycodes[0], keycodes[1] = keycodes[1], keycodes[0]
    if not sw_array[0].value:       # FWD is pressed
        keycode = keycodes[0]
    elif not sw_array[1].value:     # REV is pressed
        keycode = keycodes[1]
    elif not sw_array[2].value:     # BACK is pressed
        keycode = keycodes[2]
        time.sleep(0.5)
        if not sw_array[2].value:   # pressed long
            keycode = keycodes[3]   # Power off
    return keycode


# get sensor & switch status and return its key code
def get_keycode(int1c, sw_array):
    keycode = 0x00  # initialize
    keycode = check_sensor(int1c, keycode)
    # switch click sometimes recognized as tap
    keycode = check_switch(sw_array, keycode)  # overwrite keycode
    return keycode


# set interrupt and goto light sleep
# tls[sec]:light sleep timer for keep alive BLE
def light_sleep(tls, led_array):
    # set pin alarm. pullup for each pin is valid in light sleep
    fwd_alarm = alarm.pin.PinAlarm(pin=board.D4, value=False)
    rev_alarm = alarm.pin.PinAlarm(pin=board.D9, value=False)
    back_alarm = alarm.pin.PinAlarm(pin=board.D6, value=False)
    int1_alarm = alarm.pin.PinAlarm(pin=board.IMU_INT1, value=True)
    time_alarm = alarm.time.TimeAlarm(monotonic_time=time.monotonic() + tls)
    print('(suya~)', end='')
    led_array[3].value = LED_ON     # external LED on while light sleep
    alarm.light_sleep_until_alarms(
        fwd_alarm, rev_alarm, back_alarm, int1_alarm, time_alarm)
    led_array[3].value = LED_OFF


# deep sleep illumination (all LED turn off)
def deepsleep_led(led_array):
    for led in led_array:
        led.value = LED_ON
        time.sleep(0.1)
        led.value = LED_OFF
        time.sleep(0.1)


# set interrupt and goto pseudo deep sleep
# (true deep sleep of my XIAO nRF52840 has 2mA leak current)
# when charging, don't deep sleep for protect VBATT pin (P0.31)
def deep_sleep(ble, int1c, sw_array, led_array):
    ble_disconnection(ble)
    deepsleep_led(led_array)
    int1c.ODR_XL = 0x0  # IMU Accelerometer power down (85uA -> 3uA typ.)
    # power sw alarm needs external pullup for deep sleep although D9 in para.
    pwsw_alarm = alarm.pin.PinAlarm(pin=board.D6, value=False)
    # check charge status (board.CHARGE_STATUS = P0.17)
    sbat = digitalio.DigitalInOut(board.CHARGE_STATUS)
    sbat.direction = digitalio.Direction.INPUT
    charge_flag = sbat.value  # False: charging
    sbat.deinit()   # release the pin for set alarm
    # print('sbat.value={}. '.format(charge_flag), end='')
    if charge_flag:   # is True, do not in charge state
        print('do not charge. DEEP sleep until pwsw or start charge.', end='')
        chg_alarm = alarm.pin.PinAlarm(pin=board.CHARGE_STATUS, value=False)
        # goto pseudo deepsleep for protect battery monitor pins
        # alarm.exit_and_deep_sleep_until_alarms(pwsw_alarm, chg_alarm)
        alarm.light_sleep_until_alarms(pwsw_alarm, chg_alarm)
    else:
        print('charge now. LIGHT sleep until pwsw or stop charge.')
        chg_alarm = alarm.pin.PinAlarm(pin=board.CHARGE_STATUS, value=True)
        alarm.light_sleep_until_alarms(pwsw_alarm, chg_alarm)
        # check power switch in parallel to pwsw_alarm pin (D5+6)
        pwsw = sw_array[2].value
        if pwsw:  # if pwsw is open, wakeup with charge off
            deepsleep_led(led_array)
            print('charge finished. DEEP sleep until pwsw or start charge.')
            # change alarm logic to charge ON
            chg_alarm =\
                alarm.pin.PinAlarm(pin=board.CHARGE_STATUS, value=False)
            # goto pseudo deepsleep for protect battery monitor pins
            # alarm.exit_and_deep_sleep_until_alarms(pwsw_alarm, chg_alarm)
            alarm.light_sleep_until_alarms(pwsw_alarm, chg_alarm)
    # print('wakeup with power sw. software reset', end='')
    print('wakeup with power sw or charge on. software reset', end='')
    supervisor.reload()     # forced reboot


# send page turner actions via BLE
def pager(keycode, ms, cc, bs, rbat, led_array):
    led_array[2].value = LED_ON     # blue LED
    # set battery level before send
    bs.level = battery_percent(rbat.value, led_array)
    # send command
    if keycode == 0x76:     # double tap instead of 'Keyboard Menu'
        # goto upper left from any position (BOOX Poke Pro:1072x1448)
        # cursor should move little by little. need to tune with target reader
        for i in range(10):
            ms.move(-108, -145)
            time.sleep(0.06)    # perhaps related to scan frequency
        # goto almost center. blink LED for notificate
        led_array[2].value = LED_OFF
        for i in range(5):
            ms.move(108, 133)   # y parameter tuned with real reader
            time.sleep(0.06)
        led_array[2].value = LED_ON
        ms.click(Mouse.LEFT_BUTTON)
        ms.release_all()
        print('mouse control via bluetooth.', end='')
    else:
        cc.send(keycode)
        print('send keydata via bluetooth.', end='')
    time.sleep(0.2)     # blink BLUE LED short
    led_array[2].value = LED_OFF


# function to turn pages in e-books
# tadv[sec]:wait time for advertisement, tls[sec]:light sleep timer
def ebook_turner(tadv=60, tls=60):
    # for battery operation, shuld set p0.14 to low
    vbatt_port_guard()
    # set battery charge mode to HIGH because I use 600mAh battery
    battery_charge_mode('HIGH')
    # define pin configurations
    led_array = define_led()
    sw_array = define_switch()
    # define and initialize sensor as W-tap detector, get interrupt object
    int1c = define_sensor()
    # battery monitor
    rbat = analogio.AnalogIn(board.VBATT)   # VBATT raw R/O, 0-65535
    # bluetooth HID and Battery device description
    ble = BLERadio()
    ble.name = 'eBook_turner_w2'
    # ble.tx_power = -20    # not implemented. this app don't need 0dBm
    hid = HIDService()
    advertisement = ProvideServicesAdvertisement(hid)
    cc = ConsumerControl(hid.devices)
    ms = Mouse(hid.devices)
    bs = BatteryService()
    # initial battery level
    bs.level = battery_percent(rbat.value, led_array)
    # Disconnect if already connected for properly paring
    ble_disconnection(ble)
    # adv
    print('advertising ', end='')
    ble_advertisement(ble, advertisement, tadv, sw_array, led_array)
    if not ble.connected:
        print(' cannot connect.')
        deep_sleep(ble, int1c, sw_array, led_array)
    # key operation
    while ble.connected:
        print('\nconnected! ', end='')
        # light sleep until interrupt by key or tap
        light_sleep(tls, led_array)
        keycode = get_keycode(int1c, sw_array)
        print('keycode: 0x{:X}, '.format(keycode), end='')
        if keycode:     # is not 0x00
            pager(keycode, ms, cc, bs, rbat, led_array)
        else:
            print('there is no keycode (may wakeup by timer)', end='')
        if keycode == 0x30:    # power off
            deep_sleep(ble, int1c, sw_array, led_array)
    print('\ndisconnected')
    deep_sleep(ble, int1c, sw_array, led_array)


if __name__ == '__main__':
    tadv = 60   # wait time for advertisement in sec
    tls = 60    # light sleep timer in sec
    ebook_turner(tadv, tls)
