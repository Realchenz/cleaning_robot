#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer
import pyaudio
import json

def voice_command():
    rospy.init_node('voice_command_node', anonymous=True)
    pub = rospy.Publisher('voice_commands', String, queue_size=10)
    
    # 确保路径正确，获取当前目录并拼接模型路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(current_dir, "vosk-model-small-en-us")
    
    # 初始化 Vosk 模型
    model = Model(model_path)  # 确保此处传递的是 model 对象

    # 设置 Kaldi 识别器
    recognizer = KaldiRecognizer(model, 16000)

    audio = pyaudio.PyAudio()
    stream = audio.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=4096)
    stream.start_stream()
    
    rospy.loginfo("Starting Speech Recognition with Vosk...")

    while not rospy.is_shutdown():
        data = stream.read(4096)
        if recognizer.AcceptWaveform(data):
            result = recognizer.Result()
            text = json.loads(result)["text"]
            rospy.loginfo(f"Recognized: {text}")
            pub.publish(text)
            if "stop" in text.lower():
                rospy.loginfo("Detected command: stop")

if __name__ == '__main__':
    try:
        voice_command()
    except rospy.ROSInterruptException:
        pass
