import board
import time
from analogio import AnalogIn, AnalogOut
from digitalio import DigitalInOut, Pull, Direction
import adafruit_requests as requests
import adafruit_esp32spi.adafruit_esp32spi_socket as socket
from adafruit_esp32spi import adafruit_esp32spi
import busio
from secrets import secrets

esp32_cs = DigitalInOut(board.ESP_CS)
esp32_ready = DigitalInOut(board.ESP_BUSY)
esp32_reset = DigitalInOut(board.ESP_RESET)
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)

water_sensor = AnalogIn(board.A0)
force_sensor = AnalogIn(board.A1)
button = DigitalInOut(board.D12)
button.switch_to_input(pull=Pull.UP)
switch = DigitalInOut(board.D7)
switch.direction = Direction.OUTPUT
switch.value = False
transistorOutput = AnalogIn(board.A5)

requests.set_socket(socket, esp)

#if esp.status == adafruit_esp32spi.WL_IDLE_STATUS:
#    print("ESP32 found and in idle mode")
#print("Firmware vers.", esp.firmware_version)
#print("MAC addr:", [hex(i) for i in esp.MAC_address])

#for ap in esp.scan_networks():
#    print("\t%s\t\tRSSI: %d" % (str(ap["ssid"], "utf-8"), ap["rssi"]))

ADDR = socket.getaddrinfo("192.168.0.188", 8080)[0][-1]

class State:
    def __init__(self, interval = 0.01):
        self.interval = interval
        self.time = time.monotonic()

    def intervalPassed(self):
        return time.monotonic() - self.time > self.interval

class Sensor(State):
    def __init__(self, interval, analogInput, actionResetInterval):
        self.input = analogInput
        self.value = self.input.value
        self.actionTaken = False
        self.actionTakenTime = time.monotonic()
        self.actionResetInterval = actionResetInterval
        State.__init__(self, interval)

    def shouldTakeAction(self):
        if not(self.actionTaken) or (self.actionTaken and time.monotonic() - self.actionTakenTime > self.actionResetInterval):
            self.actionTaken = True
            self.actionTakenTime = time.monotonic()
            return True
        return False

    def intervalPassed(self):
        if time.monotonic() - self.time > self.interval:
            self.value = self.input.value
            self.time = self.time = time.monotonic()
            return True
        return False

class Network(State):
    def __init__(self, interval = 10):
        State.__init__(self, interval)

    def connectToWifi(self):
        while not esp.is_connected:
            try:
                print("Connecting to AP...")
                esp.connect_AP(secrets["ssid"], secrets["password"])
            except RuntimeError as e:
                print("could not connect to AP, retrying: ", e)
                continue
            print("Connected to", str(esp.ssid, "utf-8"), "\tRSSI:", esp.rssi)
            print("My IP address is", esp.pretty_ip(esp.ip_address))

    def sendMessage(self, recipient, subject, message):
        self.connectToWifi()
        try:
            s = socket.socket()
            s.connect(ADDR)
            s.send(bytes('%s\r\n%s\r\n%s\r\n' % (recipient, subject, message), 'utf8'))
            s.close()
        except RuntimeError as e:
            print("Could not send",recipient, subject,message,e)

    def receiveMessage(self):
        self.connectToWifi()
        try:
            s = socket.socket()
            s.connect(ADDR)
            available = s.available()
            data = s.recv(available)
            s.close()
            self.time = time.monotonic()
            return data
        except RuntimeError as e:
            print(e)
            self.time = time.monotonic()
            return None

class Planter:
    def __init__(self, moisturePollingInterval, moistureWaterThreshold, moistureWaterResetInterval, forcePollingInterval, forceTextThreshold, forceTextResetInterval, networkPollingInterval, interval=0.01):
        ### Allow these values to be updated from the raspberry pi
        self.mode = 0 # 0 for normal, 1 for logging
        self.moisturePollingInterval = moisturePollingInterval
        self.moistureWaterThreshold = moistureWaterThreshold
        self.moistureWaterResetInterval = moistureWaterResetInterval

        self.forcePollingInterval = forcePollingInterval
        self.forceTextThreshold = forceTextThreshold # Less than 1000 should send a text
        self.forceTextResetInterval = forceTextResetInterval #43200 # Twelve Hours

        self.networkPollingInterval = networkPollingInterval

        self.state = State(interval)

        self.moisture = Sensor(1, water_sensor, moistureWaterResetInterval)
        self.force = Sensor(1, force_sensor, forceTextResetInterval)
        self.network = Network(10)
        self.network.connectToWifi()

    def update(self):
        if self.state.intervalPassed():
            #print(self.force.value)
            self.state.time = time.monotonic()

        if self.force.intervalPassed():
            if self.mode == 1:
                # Implement Logging
                pass
            if self.force.shouldTakeAction() and self.force.value < self.forceTextThreshold:
                self.network.sendMessage("Primary", "Test", "Force Sensor Touched")

        if self.network.intervalPassed():
            # Send a health check message so that if the network goes bad the pi will text
            print(self.network.receiveMessage())
            # Update parameters or mode based on messages from pi
            self.network.time = time.monotonic()


planter = Planter(
    moisturePollingInterval = 1, # Minimum number of seconds between sensor readings
    moistureWaterThreshold = 30000, # A moisture value over this will turn on the pump either for a certain period of time or until the threshold is re-crossed
    moistureWaterResetInterval = 43200, # The pump cannot turn back on until this many seconds have passed (to prevent overwatering)
    forcePollingInterval = 1, # Minimum number of seconds between sensor readings
    forceTextThreshold = 1000, # A force value lower than this will trigger a text every forceTextResetInterval seconds until resolved
    forceTextResetInterval = 43200, # Twelve Hours
    networkPollingInterval = 10,
    interval=0.01
)
#connectToWifi()
while True:
    #switch.value = False
    #print("Off", water_sensor.value, force_sensor.value)
    #while not button.value:
    #    switch.value = True
    #    print("On, button")
    #    time.sleep(0.1)
    #while water_sensor.value > 100000:
    #    switch.value = True
    #    print("On, water")
    #    time.sleep(0.1)

    planter.update()
    #time.sleep(0.1)