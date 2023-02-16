'''
eBook_turner_w2 Android用電子書籍ページめくり機 BLE版 ver.2 by @pado3
target device: Seeed studio XIAO nRF52840 Sense (w/LSM6DS3TR-C)

r1.0  2023/02/15 initial release
r1.0a 2023/02/15 without double tap function (may work with /Sense)

Reader&KindleはUPで戻りDOWNで送る。Kinoppyは逆。読書尚友&なろうリーダは任意。
FWD:D3+4, REV:D8+9, BACK/PW:D5+6, mode:D7 (3,8,5,7が入力、4,9,6が割り込み)
D5はDeep sleep中も監視するため外部プルアップ(100k)
D7はKinoppyモードで常時LOWに引くため内部13kではリーク大きく外部プルアップ(100k)
外部LED Anode:D1(常時H), Kathode:D0(XIAO内蔵LEDと同じく逆論理)
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


# LED logic of XIAO is reversed and often confusing, so make clarify
LED_ON = False
LED_OFF = True


# protect VBATT from over voltage in battery operation (>3.6V)
# cannot set low during deep sleep. Do not fall into deep sleep while charging.
def vbatt_port_guard():
    # set P0.14 to LOW
    ebat = digitalio.DigitalInOut(board.READ_BATT_ENABLE)
    ebat.direction = digitalio.Direction.OUTPUT
    ebat.value = False  # should set low before ADC or until charge
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
        board.D0,           # additional LED with low current (10k, 0.15mA)
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


# calculate battery level in percent and set low battery LED
def battery_percent(readout, led_array):
    vbat = readout * (3300 / 65536) * (1510 / 510)
    pc = int(100 * (readout - 21400) / (23900 - 21400))
    print('VBATT:{:.0f}mV, {}%, '.format(vbat, pc), end='')
    if pc > 100:
        pc = 100
    elif pc < 0:
        pc = 0
    if pc < 20:     # below 20% (~3.3V), RED led always ON
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


# get switch status and return its key code
def get_keycode(sw_array):
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
    keycode = 0x00  # initialize
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


# set interrupt and goto light sleep
# tls[sec]:light sleep timer for keep alive BLE
def light_sleep(tls, led_array):
    # set pin alarm. pullup for each pin is valid in light sleep
    fwd_alarm = alarm.pin.PinAlarm(pin=board.D4, value=False)
    rev_alarm = alarm.pin.PinAlarm(pin=board.D9, value=False)
    back_alarm = alarm.pin.PinAlarm(pin=board.D6, value=False)
    time_alarm = alarm.time.TimeAlarm(monotonic_time=time.monotonic() + tls)
    print('(suya~)', end='')
    led_array[3].value = LED_ON     # external led on while light sleep
    alarm.light_sleep_until_alarms(
        fwd_alarm, rev_alarm, back_alarm, time_alarm)
    led_array[3].value = LED_OFF


# deep sleep illumination (all LED turn off)
def deepsleep_led(led_array):
    for led in led_array:
        led.value = LED_ON
        time.sleep(0.1)
        led.value = LED_OFF
        time.sleep(0.1)


# set interrupt and goto pseudo deep sleep
# (true deep sleep of XIAO nRF52840 has 2mA leak current)
# when charging, don't deep sleep for protect VBATT pin (P0.31)
def deep_sleep(ble, sw_array, led_array):
    deepsleep_led(led_array)
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
        # goto pseudo deepsleep
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
            # goto pseudo deepsleep
            # alarm.exit_and_deep_sleep_until_alarms(pwsw_alarm, chg_alarm)
            alarm.light_sleep_until_alarms(pwsw_alarm, chg_alarm)
    # print('wakeup with power sw. software reset', end='')
    print('wakeup with power sw or charge on. software reset', end='')
    supervisor.reload()     # forced reboot


# turner actions
def pager(keycode, cc, bs, rbat, ble, led_array):
    led_array[2].value = LED_ON
    cc.send(keycode)
    print('send keydata via bluetooth.', end='')
    bs.level = battery_percent(rbat.value, led_array)
    time.sleep(0.1)     # blink BLUE LED short
    led_array[2].value = LED_OFF


# function to turn pages in e-books
# tadv[sec]:wait time for advertisement, tls[sec]:light sleep timer
def ebook_turner(tadv=60, tls=60):
    # set P0.14 to LOW before battery monitor
    vbatt_port_guard()
    # set battery charge mode to HIGH because this use 600mAh battery
    battery_charge_mode('HIGH')
    # define pin configurations
    led_array = define_led()
    sw_array = define_switch()
    # battery monitor
    rbat = analogio.AnalogIn(board.VBATT)   # VBATT R/O, 0-65535
    # bluetooth HID and Battery device description
    ble = BLERadio()
    ble.name = 'eBook_turner_w2'
    # ble.tx_power = -20    # not implemented. this app don't need 0dBm
    hid = HIDService()
    advertisement = ProvideServicesAdvertisement(hid)
    cc = ConsumerControl(hid.devices)
    bs = BatteryService()
    # battery check in startup
    bs.level = battery_percent(rbat.value, led_array)
    # Disconnect if already connected for properly paring
    ble_disconnection(ble)
    # adv
    print('advertising ', end='')
    ble_advertisement(ble, advertisement, tadv, sw_array, led_array)
    if not ble.connected:
        print(' cannot connect.')
        deep_sleep(ble, sw_array, led_array)
    # key operation
    while ble.connected:
        print('\nconnected! ', end='')
        keycode = get_keycode(sw_array)
        print('keycode: 0x{:X}, '.format(keycode), end='')
        if keycode:     # keycode is not 0x00
            pager(keycode, cc, bs, rbat, ble, led_array)
        else:
            print('there is no keycode')
        if keycode == 0x30:    # power off
            ble_disconnection(ble)
            deep_sleep(ble, sw_array, led_array)
        light_sleep(tls, led_array)
    print('\ndisconnected')
    deep_sleep(ble, sw_array, led_array)


if __name__ == '__main__':
    tadv = 60   # wait time for advertisement in sec
    tls = 60    # light sleep timer in sec
    ebook_turner(tadv, tls)
