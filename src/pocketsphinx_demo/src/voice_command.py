#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import speech_recognition as sr # type: ignore

def voice_command():
    rospy.init_node('voice_command_node', anonymous=True)
    pub = rospy.Publisher('voice_commands', String, queue_size=10)
    
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    
    rospy.loginfo("Starting SpeechRecognition with PocketSphinx...")

    while not rospy.is_shutdown():
        with mic as source:
            rospy.loginfo("Listening for command...")
            recognizer.adjust_for_ambient_noise(source, duration=0.5)  # 缩短环境噪声调整时间
            print("======================================")
            print("RECOGNIZING MODE.....")
            print("======================================\n")
            
            try:
                # 设置监听超时时间为1秒，最长语音持续时间为2秒
                audio = recognizer.listen(source, timeout=1, phrase_time_limit=2)
                recognized_text = recognizer.recognize_sphinx(audio)
                rospy.loginfo(f"Recognized: {recognized_text}")
                pub.publish(recognized_text)

                # Only for test, use subscriber to get the recognized text
                if "stop" in recognized_text.lower():  # 转小写以便不区分大小写匹配
                    print("\n发现单词stop\n\n")
                    print("======================================\n")
                else:
                    print("\n未发现单词stop\n\n")
                    print("======================================\n")






            except sr.WaitTimeoutError:
                rospy.logwarn("Listening timed out while waiting for phrase to start")
            except sr.UnknownValueError:
                rospy.logwarn("PocketSphinx could not understand audio")
            except sr.RequestError as e:
                rospy.logwarn(f"Could not request results from PocketSphinx; {e}")

if __name__ == '__main__':
    try:
        voice_command()
    except rospy.ROSInterruptException:
        pass
