"""Demo of using the python sound_play client.  This will only work
if the sound_play node is running.
"""
import rospy

def demo():
    from sound_play.libsoundplay import SoundClient
    
    rospy.init_node('talker')
    soundhandle = SoundClient()
    
    while not rospy.is_shutdown():
        s1 = soundhandle.voiceSound("Happy holloween")
        s1.play()
        rospy.sleep(5)

if __name__ == "__main__":
    demo()
