#!/usr/bin/python
# -*- coding: utf-8 -*-

import requests
import json
import pyaudio
import wave
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

from keys import wit_access_token
from threading import Thread

import rospy
from std_msgs.msg import String

# Wit speech API endpoint
API_ENDPOINT = 'https://api.wit.ai/speech'

class MyThread(Thread):
    def __init__(self, audio):
        Thread.__init__(self)
        self.audio = audio

    def run(self):
        headers = {'authorization': 'Bearer ' + wit_access_token,
                'Content-Type': 'audio/wav'}

        try:
            resp = dict(requests.post(API_ENDPOINT, 
                headers = headers,
                data = self.audio).json())
            reco.data = "Operator: {}".format(resp[u'_text'])
            reco_pub.publish(reco)
        except:
            pass


def record_audio(RECORD_SECONDS, WAVE_OUTPUT_FILENAME):
    #--------- SETTING PARAMS FOR OUR AUDIO FILE ------------#
    FORMAT = pyaudio.paInt16    # format of wave
    CHANNELS = 2                # no. of audio channels
    RATE = 8000                # frame rate
    CHUNK = 1024                # frames per audio sample
    #--------------------------------------------------------#

    # creating PyAudio object
    audio = pyaudio.PyAudio()

    # open a new stream for microphone
    # It creates a PortAudio Stream Wrapper class object
    stream = audio.open(format=FORMAT,channels=CHANNELS,
                        rate=RATE, input=True,
                        frames_per_buffer=CHUNK)

    #----------------- start of recording -------------------#
    print("Listening...")

    # list to save all audio frames
    frames = []

    for i in range(int(RATE / CHUNK * RECORD_SECONDS)):
        # read audio stream from microphone
        data = stream.read(CHUNK)
        # append audio data to frames list
        frames.append(data)

    #------------------ end of recording --------------------#
    print("Finished recording.")

    stream.stop_stream()    # stop the stream object
    stream.close()          # close the stream object
    audio.terminate()       # terminate PortAudio

    #------------------ saving audio ------------------------#

    # create wave file object
    waveFile = wave.open(WAVE_OUTPUT_FILENAME, 'wb')

    # settings for wave file object
    waveFile.setnchannels(CHANNELS)
    waveFile.setsampwidth(audio.get_sample_size(FORMAT))
    waveFile.setframerate(RATE)
    waveFile.writeframes(b''.join(frames))

    # closing the wave file object
    waveFile.close()

def read_audio(WAVE_FILENAME):
    # function to read audio(wav) file
    with open(WAVE_FILENAME, 'rb') as f:
        audio = f.read()
    return audio

def recognize_speech(AUDIO_FILENAME, num_seconds = 5):
    record_audio(num_seconds, AUDIO_FILENAME)
    audio = read_audio(AUDIO_FILENAME)
    t = MyThread(audio)
    t.start()


if __name__ == "__main__":
    rospy.init_node("recognizer", anonymous=True)
    reco_pub = rospy.Publisher("/recognized", String, queue_size=1)
    reco = String()
    while True:
        recognize_speech('myspeech.wav', 8)
