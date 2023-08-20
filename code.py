# SPDX-FileCopyrightText: 2023 ladyada for Adafruit Industries
#
# SPDX-License-Identifier: MIT

# Demo audio player that plays random wav files from internal storage and
# SD card. Default pinout matches the Audio BFF for QT Py S2, S3 and RP2040
import gc
import time
import json
import os
import random
import audiocore
import board
import audiobusio
import audiomixer
import adafruit_sdcard
import storage
import digitalio
import touchio
import asyncio
import busio
import ulab.numpy as np

from vl53l5cx import DATA_TARGET_STATUS, DATA_DISTANCE_MM, DATA_MOTION_INDICATOR
from vl53l5cx import STATUS_VALID, RESOLUTION_4X4, RESOLUTION_8X8, RANGING_MODE_CONTINUOUS
from vl53l5cx.cp import VL53L5CXCP

DEBUG = False
                        
I2C_FREQUENCY=1_000_000
CURRENT_VOLUME = 0.5
AUDIO_BUFFER_SIZE = 1024

FRAMERATE = 5 # Hz
RESOLUTION = RESOLUTION_8X8
SHARPERNER = 20 # None # percentage sharpener if desired. None for default

touch = None # this can be defined at the board level just beneath...

card_cs = digitalio.DigitalInOut(board.A0)
card_cs.direction = digitalio.Direction.INPUT
card_cs.pull = digitalio.Pull.UP
sdcard = None

if(board.board_id == "adafruit_feather_rp2040_prop_maker"):
    audio = audiobusio.I2SOut(board.I2S_BIT_CLOCK, board.I2S_WORD_SELECT, board.I2S_DATA)
elif (board.board_id == "adafruit_qtpy_esp32s2"):
    touch = touchio.TouchIn(board.TX)
    DATA = board.A1
    LRCLK = board.A2
    BCLK = board.A3
    audio = audiobusio.I2SOut(BCLK, LRCLK, DATA)
else: #PCM5102
    i2s_bck_pin = board.MOSI # PCM5102 BCK pin
    i2s_lck_pin = board.MISO # PCM5102 LCK pin
    i2s_dat_pin = board.SCK  # PCM5102 DIN pin
    audio = audiobusio.I2SOut(bit_clock=i2s_bck_pin, 
                            word_select=i2s_lck_pin, 
                            data=i2s_dat_pin)
mixer = None

button = digitalio.DigitalInOut(board.BUTTON)
button.switch_to_input(pull=digitalio.Pull.UP)

wave_files = []
for filename in sorted(os.listdir("/sounds")):
    filename = filename.lower()
    if filename.endswith(".wav") and not filename.startswith("."):
        wave_files.append("/sounds/" + filename)

def button_pressed():
    # print("button check, button.value",not button.value, "touch.value", touch.value if touch else "no touch")
    return (not button.value) or (touch and touch.value)

async def open_audio():
    n = random.choice(wave_files)
    print("playing", n)
    f = open(n, "rb")
    w = audiocore.WaveFile(f)
    return f, w


wavefile = 0

print(json.dumps(wave_files))

last_printed_time = time.monotonic() - 9
last_distance_reading_time = time.monotonic() - 9

if not sdcard:
    try:
        sdcard = adafruit_sdcard.SDCard(board.SPI(), card_cs)
        vfs = storage.VfsFat(sdcard)
        storage.mount(vfs, "/sd")
        print("Mounted SD card")
        wave_files = [
            "/sounds/" + file
            for file in os.listdir("/sounds")
            if file.endswith(".wav")
        ]
        wave_files += [
            "/sd/" + file for file in os.listdir("/sd") if file.endswith(".wav")
        ]
        print(wave_files)
    except OSError:
        print("No SD card access failure")
        pass


i2c = None
grid_width_zero_indexed, GRID_WIDTH, GRID_CELLS = 3, 4, 16

PREVIOUS_DISTANCE_READINGS = [[]]
MAX_PREVIOUS_DISTANCE_READINGS = max(int(FRAMERATE * 3), 5) # at least 3 previous readings for avg / slope calculation.
print("MAX Previous Readings set to",MAX_PREVIOUS_DISTANCE_READINGS)

def make_sensors(FREQUENCY=1_000_000):
    global I2C_FREQUENCY, PREVIOUS_DISTANCE_READINGS
    # lpn = low power pin, removed as using the micro VL53L5CX breakout without lpn
    # scl_pin, sda_pin = (board.SCL, board.SDA)

    #i2c = busio.I2C(board.SCL1, board.SDA1,frequency=FREQUENCY)
    # i2c = busio.I2C(scl_pin, sda_pin, frequency=FREQUENCY)
    # i2c = board.STEMMA_I2C()
    I2C_FREQUENCY = FREQUENCY
    if hasattr(board, "SCL1"):
        PREVIOUS_DISTANCE_READINGS = [[],[]]
        return [VL53L5CXCP(busio.I2C(board.SCL,board.SDA,frequency=FREQUENCY)), VL53L5CXCP(busio.I2C(board.SCL1,board.SDA1,frequency=FREQUENCY))]
    return [VL53L5CXCP(busio.I2C(board.SCL,board.SDA,frequency=FREQUENCY))]


def configure_sensor(tof=None,resolution = None):
    global I2C_FREQUENCY, GRID_CELLS, GRID_WIDTH, SHARPERNER
    if(tof is None):
        raise ValueError("tof argument is None, expected VL53L5CX type thing.")
    print("resetting sensor")
    tof.reset()

    if not tof.is_alive():
        raise ValueError("VL53L5CX not detected")
    print(tof)
    print("initting sensor - sending firmware")
    if I2C_FREQUENCY < 1_000_000:
        print("*** I2C bus is setup to be slow, change by manually initialising with 1MHZ or whatever suits to speed up firmware upload and data receive rate ***")
    tof.init()

    print("setting resolution")
    tof.resolution = resolution if resolution else RESOLUTION_8X8
    
    grid_width_zero_indexed = float(7 if tof.resolution == RESOLUTION_8X8 else 3)
     # auto changes to 3 if 4x4 grid
    GRID_WIDTH = grid_width_zero_indexed + 1
    GRID_CELLS = GRID_WIDTH * GRID_WIDTH

    print("setting ranging frequency", end="")
    tof.ranging_freq = 15 # was 2, 4 felt intense! Suggest 15 for 8x8
    print(" - set to",tof.ranging_freq)

    if SHARPERNER is not None:
        print("Setting ToF Sharpening to {:d}%".format(SHARPERNER))
        tof.sharpener_percent = SHARPERNER

    print("starting ranging...", end="")
    # tof.ranging_mode = RANGING_MODE_CONTINUOUS
    # tof.sharpener_percent = 20
    tof.start_ranging({DATA_DISTANCE_MM, DATA_TARGET_STATUS})#, DATA_MOTION_INDICATOR})
    print("done")
    return tof


def add_distances_to_previous_readings(dist_matrix, valid_matrix, previous_readings):
    global MAX_PREVIOUS_DISTANCE_READINGS, DEBUG
    results = np.where(valid_matrix, dist_matrix, 0)
    if DEBUG:
        print("Adding distance matrix to previous readings", dist_matrix.tolist())
        print("valid_matrix.count(True)", valid_matrix.tolist().count(True))
        print(results.tolist())
    previous_readings.insert(0, results)
    if(len(previous_readings) > MAX_PREVIOUS_DISTANCE_READINGS):
        previous_readings = previous_readings[:MAX_PREVIOUS_DISTANCE_READINGS]

def get_distances_and_valid_targets(sensor_data: VL53L5CXCP.Results, previous_readings=None):
    global GRID_WIDTH
    if previous_readings is None:
        raise ValueError("previous_readings argument is None, expected list of previous readings.")
    else:
        if(DEBUG):
            print("previous_readings",previous_readings)
    distance_data = np.ndarray(sensor_data.distance_mm, dtype=np.uint16)
    if(DEBUG):
        print("data",distance_data)
    distance_matrix = distance_data.reshape((int(GRID_WIDTH),int( GRID_WIDTH)))
    if(DEBUG):
        print("matrix)",distance_matrix)
    # data_buffer=np.zeros((int(GRID_WIDTH),int( GRID_WIDTH)), dtype=np.uint16)
    valid_data = np.ndarray(sensor_data.target_status, dtype=np.uint16).reshape((int(GRID_WIDTH),int( GRID_WIDTH)))
    if(DEBUG):
        print("data",valid_data)
    valid_matrix = valid_data == STATUS_VALID ## Use array creation based on condition, if needed call .astype(int) on it
    if(DEBUG):
        print("matrix)",valid_matrix)
    add_distances_to_previous_readings(distance_matrix,valid_matrix, previous_readings)
    return distance_matrix, valid_matrix

def apply_voting(sensor, num_readings):
    data = np.zeros((8, 8, num_readings))
    for i in range(num_readings):
        for j in range(8):
            sensor.get_measurement(data[j, :, i])
    modes = np.zeros((8, 8))
    for i in range(8):
        for j in range(8):
            hist, _ = np.histogram(data[i, j, :], bins=range(65536))
            modes[i, j] = np.argmax(hist)
    return modes


# Process modes as needed


# notes, long press = switch half and full grid (GRID_MODES)
# short press = switch ALT_SIDE_MODES aka synth modes
# super long press (5s) = calibration mode
# calibration mode = set min and max distance for the right hand (whichever is pitch/volume) half, save to file
## In calibration mode, have always calibrate calibration


# Mode variables
ALT_SIDE_MODES = ["Pitch & Volume", "Drum Kit"]#, "Sample Bank 1", "Sample Bank 2 of 7" "Harmonic Modulation", "Filter Control", "Delay Effects"]
CURRENT_MODE = 1
CALIBRATION_STAGE = 0
IS_CALIBRATED = False
# load calibration from file or nvm etc.


class GRID_MODES(dict):
    FULL_GRID_AS_PITCH_VOLUME_4X4 = 0
    FULL_GRID_AS_PITCH_VOLUME_8X8 = 1
    RIGHT_HALF_PITCH_VOLUME_4X4 = 2
    RIGHT_HALF_PITCH_VOLUME_8X8 = 3
    LEFT_HALF_PITCH_VOLUME_4X4 = 4
    LEFT_HALF_PITCH_VOLUME_8X8 = 5
    TOP_HALF_PITCH_VOLUME_4X4 = 6
    TOP_HALF_PITCH_VOLUME_8X8 = 7
    BOTTOM_HALF_PITCH_VOLUME_4X4 = 8
    BOTTOM_HALF_PITCH_VOLUME_8X8 = 9
    # RIGHT_HAND_SIDE_PITCH_VOLUME_LEFT_HALF_DRUMKIT_8X8 = 2 # 10
MODES = GRID_MODES

async def get_distances(tof=None, previous_readings=None):
    global i2c, I2C_FREQUENCY, grid_width_zero_indexed, GRID_WIDTH, GRID_CELLS
    if(tof is None):
        raise ValueError("tof argument is None, expected VL53L5CX type thing.")
    if(previous_readings is None):
        raise ValueError("previous_readings argument is None, expected list of previous readings.")
    while True:
        if tof.check_data_ready():
            print("data ready, getting ranging data...")
            results = tof.get_ranging_data()
            distances_data = get_distances_and_valid_targets(results, previous_readings)
            distance = results.distance_mm
            status = results.target_status
            for i, d in enumerate(distance):
                if status[i] == STATUS_VALID:
                    if(DEBUG):
                        print("{:2d}:{:-4d}\t".format(i,d), end="")
                else:
                    if(DEBUG):
                        print("{:2d}:   0\t".format(i), end="")
                if(((float(i+1) / (grid_width_zero_indexed+1)) - int(float(i+1)/ (grid_width_zero_indexed+1))) * (grid_width_zero_indexed+1) == 0 and i != 0):
                    if(DEBUG):
                        print("")
                else:
                    pass
            if(DEBUG):
                print("")
            return distances_data
        else:
            print("-", end="")
            return (None,)
            #TODO: fix this as a single sensor failure will infinitely loop probably.


tof_sensors = make_sensors(I2C_FREQUENCY)
for tof in tof_sensors:
    configure_sensor(tof, RESOLUTION)

CURRENT_READING = -1
CSV_PATH = None

async def main():
    global wavefile, mixer, audio, button, touch, last_printed_time, last_distance_reading_time, CURRENT_READING, FRAMERATE, PREVIOUS_DISTANCE_READINGS, tof_sensors, CSV_PATH
    while True:
        CURRENT_READING = (CURRENT_READING + 1) % MAX_PREVIOUS_DISTANCE_READINGS
        if button_pressed():
            if mixer and mixer.voice[0].playing:
                print("stopping")
                mixer.voice[0].stop()
                if wavefile:
                    wavefile.close()
            else:
                wavefile, wave = await open_audio()
                print("wavefile", json.dumps(wavefile))
                print("sample_rate",json.dumps(wave.sample_rate))
                print("bits_per_sample",json.dumps(wave.bits_per_sample))
                print("channel_count",json.dumps(wave.channel_count))
                try:
                    mixer = audiomixer.Mixer(
                        voice_count=1,
                        buffer_size=AUDIO_BUFFER_SIZE,
                        sample_rate=wave.sample_rate,
                        channel_count=wave.channel_count,
                        bits_per_sample=wave.bits_per_sample,
                        samples_signed=True,
                    )
                    mixer.voice[0].level = CURRENT_VOLUME
                    audio.play(mixer)
                    mixer.voice[0].play(wave)
                except ValueError as e:
                    if e.args and e.args.count(
                        "The sample's signedness does not match the mixer's"
                    ):
                        mixer = audiomixer.Mixer(
                            voice_count=1,
                            buffer_size=AUDIO_BUFFER_SIZE,
                            sample_rate=wave.sample_rate,
                            channel_count=wave.channel_count,
                            bits_per_sample=wave.bits_per_sample,
                            samples_signed=False,
                        )
                        print("Failed to load signed samples, Retrying with unsigned mixer samples")
                        mixer.voice[0].level = CURRENT_VOLUME
                        audio.play(mixer)
                        mixer.voice[0].play(wave)
                    else:
                        print("Failed to load sample, cannot play it if we can't load it. Skipping")
                        print(json.dumps(e,))

            while button_pressed():
                pass

        elif (time.monotonic() - last_printed_time) >=2:
            last_printed_time = time.monotonic()
            print(
                "still playing"
                if mixer and mixer.voice[0].playing
                else "waiting for button 0"
            )
            pass

        if(time.monotonic() - last_distance_reading_time) >=1/FRAMERATE:
            last_distance_reading_time = time.monotonic()
            print("***************reading distance")
            for i in range(len(tof_sensors)):
                if(DEBUG):
                    print("previous readings",PREVIOUS_DISTANCE_READINGS[i])
                await get_distances(tof_sensors[i],PREVIOUS_DISTANCE_READINGS[i])
            print("***************done")

        if(CURRENT_READING == MAX_PREVIOUS_DISTANCE_READINGS-1):
            print("current frame", CURRENT_READING)
            DATA_TO_WRITE = PREVIOUS_DISTANCE_READINGS
            print("Writing to Storage, ", end="")
            if(sdcard and not CSV_PATH):
                CSV_PATH  = "/sd/" + str(time.monotonic_ns()) + ".csv"
                print("SD Card")
            elif(not CSV_PATH):
                CSV_PATH = "/" + str(time.monotonic_ns()) + ".csv"
                print("Internal Flash")
            with open(CSV_PATH, "a+") as f:
                print("Writing to", CSV_PATH)
                f.write("Timestamp, " + json.dumps(DATA_TO_WRITE))
                f.flush()
                f.close()
            print("Done Writing to Storage, garbage collecting...")
            gc.collect()
            print("Done garbage collecting")
        else:
            print("current frame", CURRENT_READING, "skipping write")



        #await asyncio.sleep(1/FRAMERATE)

asyncio.run(main())


# ###################################################
# # This code sets up the I2C bus and the VL53L4X sensor, and an I2S audio output on pins D1, D2, and D3. It then enters a loop where it reads the distance from the sensor, calculates the pitch and volume based on the X and Y position of the hand, and sets the frequency and volume of the audio output accordingly. It then checks the position of the hand and controls the audio playback accordingly. If the hand is in the top left corner, it pauses or resumes the current sample. If the hand is in the middle left, it plays the previous or next sample. If the hand is in the bottom left, it starts or stops all active samples. The loop then waits for a short time before reading the distance again. 
# import board
# import time
# import adafruit_vl53l4x
# import audioio
# import os

# # Set up the I2C bus and the VL53L4X sensor
# i2c = board.I2C()
# sensor = adafruit_vl53l4x.VL53L4X(i2c)

# # Set up the I2S audio output
# audio = audioio.I2SOut(board.D1, board.D2, board.D3)

# # Set up the loop to read the distance from the sensor and control audio playback
# while True:
#     # Read the distance from the sensor and calculate the pitch and volume
#     x_distance = sensor.range
#     y_distance = sensor.range + 50
#     pitch = 440 + (x_distance - 100) * 10
#     volume = max(0, min(1, (y_distance - 100) / 200))
    
#     # Set the frequency and volume of the audio output
#     audio.frequency = int(pitch)
#     audio.volume = volume
    
#     # Check the position of the hand and control the audio playback accordingly
#     if x_distance < 100:
#         # Pause or play the current sample in the top left corner
#         if pitch < 600:
#             if audio.playing:
#                 audio.pause()
#             else:
#                 audio.resume()
#         # Play the previous or next sample in the middle left
#         elif pitch < 800:
#             if pitch < 700:
#                 os.system("mpg321 /sd/previous.mp3")
#             else:
#                 os.system("mpg321 /sd/next.mp3")
#         # Start or stop all active samples in the bottom left
#         else:
#             if pitch < 1000:
#                 os.system("mpg321 /sd/start.mp3")
#             else:
#                 os.system("mpg321 /sd/stop.mp3")
    
#     # Wait a short time before reading the distance again
#     time.sleep(0.01)
    
# ###################################################

# import board
# import time
# import adafruit_vl53l4x
# import audioio
# import os

# # Set up the I2C bus and the VL53L4X sensor
# i2c = board.I2C()
# sensor = adafruit_vl53l4x.VL53L4X(i2c)

# # Set up the I2S audio output
# audio = audioio.I2SOut(board.D1, board.D2, board.D3)

# # Set up the loop to read the distance from the sensor and control audio playback
# while True:
#     # Read the distance from the sensor and calculate the pitch and sample control
#     distance = sensor.range
#     pitch = 440 + (distance - 100) * 10
    
#     # Check the position of the hand and control the audio playback accordingly
#     if distance < 100:
#         # Pause or play the current sample in the top left corner
#         if pitch < 600:
#             if audio.playing:
#                 audio.pause()
#             else:
#                 audio.resume()
#         # Play the previous or next sample in the middle left
#         elif pitch < 800:
#             if pitch < 700:
#                 os.system("mpg321 /sd/previous.mp3")
#             else:
#                 os.system("mpg321 /sd/next.mp3")
#         # Start or stop all active samples in the bottom left
#         else:
#             if pitch < 1000:
#                 os.system("mpg321 /sd/start.mp3")
#             else:
#                 os.system("mpg321 /sd/stop.mp3")
#     else:
#         # Set the frequency of the audio output to the pitch
#         audio.frequency = int(pitch)
    
#     # Wait a short time before reading the distance again
#     time.sleep(0.01)
    
# ###################################################

# import board
# import time
# import adafruit_vl53l4x
# import audioio
# import os
# import sdioio
# import storage

# # Mount the SD card
# sd = sdioio.SDCard(board.SDIO_DATA0, board.SDIO_DATA1, board.SDIO_DATA2, board.SDIO_DATA3, board.SDIO_CLOCK)
# vfs = storage.VfsFat(sd)
# storage.mount(vfs, "/sd")

# # Set up the I2C bus and the VL53L4X sensor
# i2c = board.I2C()
# sensor = adafruit_vl53l4x.VL53L4X(i2c)

# # Set up the I2S audio output
# audio = audioio.I2SOut(board.D1, board.D2, board.D3)

# # Set up the loop to read the distance from the sensor and play the audio file
# while True:
#     # Read the distance from the sensor and calculate the frequency
#     distance = sensor.range
#     freq = 440 + (distance - 100) * 10
    
#     # Set the frequency of the audio output
#     audio.frequency = int(freq)
    
#     # Check for a WAV file on the SD card
#     for file in os.listdir("/sd"):
#         if file.endswith(".wav"):
#             # Open the WAV file and play it on the audio output
#             with open("/sd/" + file, "rb") as f:
#                 wav = audioio.WaveFile(f)
#                 audio.play(wav)
#                 while audio.playing:
#                     pass
#             break
    
#     # Wait a short time before reading the distance again
#     time.sleep(0.01)


###################################################

# import board
# import time
# import adafruit_vl53l4x
# import audioio
# import os

# # Set up the I2C bus and the VL53L4X sensor
# i2c = board.I2C()
# sensor = adafruit_vl53l4x.VL53L4X(i2c)

# # Set up the I2S audio output
# audio = audioio.I2SOut(board.D1, board.D2, board.D3)

# # Set up the loop to read the distance from the sensor and play the audio file
# while True:
#     # Read the distance from the sensor and calculate the frequency
#     distance = sensor.range
#     freq = 440 + (distance - 100) * 10
    
#     # Set the frequency of the audio output
#     audio.frequency = int(freq)
    
#     # Check for a WAV file on the SD card
#     for file in os.listdir("/sd"):
#         if file.endswith(".wav"):
#             # Open the WAV file and play it on the audio output
#             with open("/sd/" + file, "rb") as f:
#                 wav = audioio.WaveFile(f)
#                 audio.play(wav)
#                 while audio.playing:
#                     pass
#             break
    
#     # Wait a short time before reading the distance again
#     time.sleep(0.01)


###################################################

# import board
# import time
# import pulseio
# import adafruit_vl53l4x

# # Set up the I2C bus and the VL53L4X sensor
# i2c = board.I2C()
# sensor = adafruit_vl53l4x.VL53L4X(i2c)

# # Set up the PWM output for the speaker
# speaker = pulseio.PWMOut(board.D5, duty_cycle=0, frequency=440, variable_frequency=True)

# # Set up the loop to read the distance from the sensor and adjust the frequency of the speaker
# while True:
#     # Read the distance from the sensor and calculate the frequency
#     distance = sensor.range
#     freq = 440 + (distance - 100) * 10
    
#     # Set the frequency of the speaker
#     speaker.frequency = int(freq)
    
#     # Wait a short time before reading the distance again
#     time.sleep(0.01)