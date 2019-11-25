import argparse
import math
import time
import motion
from naoqi import ALProxy

def hello_ID(tts, motionProxy, postureProxy):

    postureProxy.goToPosture("StandInit",0.5)

    # Wake up robot
    motionProxy.wakeUp()
    
    # Joint matrix for the hello movement
    JointNamesL = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"]
    JointNamesL2 = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand", "HeadYaw", "HeadPitch"]
    
    # waving right
    Arm1 = [-70,  40, 0, -35, 20, 50]
    Arm1 = [ x * motion.TO_RAD for x in Arm1]
    
    # waving left
    Arm2 = [-70,  40, 0, -75, -20, 50]
    Arm2 = [ x * motion.TO_RAD for x in Arm2]
    
    Arm3 = [-70,  40, 0, -35, 20, 50, -20, -10]
    Arm3 = [ x * motion.TO_RAD for x in Arm3]
    tts.post.say("Hello, I am BEER BOT, if you would like a beer,")
    
    pFractionMaxSpeed = 0.35
    pFractionMaxSpeed1 = 0.2
    
    motionProxy.post.angleInterpolationWithSpeed(JointNamesL, Arm1, pFractionMaxSpeed)
    motionProxy.angleInterpolationWithSpeed(JointNamesL, Arm2, pFractionMaxSpeed)
    motionProxy.angleInterpolationWithSpeed(JointNamesL, Arm1, pFractionMaxSpeed)
    motionProxy.angleInterpolationWithSpeed(JointNamesL, Arm2, pFractionMaxSpeed)
    motionProxy.angleInterpolationWithSpeed(JointNamesL2, Arm3, pFractionMaxSpeed1)
    
    # Joint matrix for the ID movement
    ID = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand","RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand", "HeadYaw", "HeadPitch"]
    
    # pointing at the ID reader
    ID_position = [30,  -20, 0, -60, 0, 50,70,-50, 0, 0, 50, 50,-50,10]
    ID_position = [ x * motion.TO_RAD for x in ID_position]
    
    # looking at the customer
    ID_position2 = [30,  -20, 0, -60, 0, 50,70,-50, 0, 0, 50, 50,0,-20]
    ID_position2 = [ x * motion.TO_RAD for x in ID_position2]
    
    pFractionMaxSpeed = 0.3
    
    tts.post.say("Please put your ID here, ")
    
    motionProxy.post.angleInterpolationWithSpeed(ID, ID_position, pFractionMaxSpeed)
    
    tts.post.say("So I can verify that you are over 18!")
    
    motionProxy.angleInterpolationWithSpeed(ID, ID_position2, pFractionMaxSpeed)
    
    
def drink_select(tts, motionProxy, postureProxy, above18):
    # Joint matrix for the touchpad movement
    Touchpad = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand","RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand", "HeadYaw", "HeadPitch"]
    
    # Pointing to the touchpad
    Touchpad_position = [70,  50, 0, 0, -50, 50, 30, 20, 0, 60, 0, 50, 50,10]
    Touchpad_position = [ x * motion.TO_RAD for x in Touchpad_position]
    
    # tilting its head
    Touchpad_position2 = [70,  50, 0, 0, -50, 50, 30, 0, 80, 20, 50, 50, 0,0]
    Touchpad_position2 = [ x * motion.TO_RAD for x in Touchpad_position2]
    
    # Opening other arm to list the beers
    Touchpad_position3 = [70,  50, 0, 0, -50, 50, 30, 0, 80, 20, 50, 50, -20,-20]
    Touchpad_position3 = [ x * motion.TO_RAD for x in Touchpad_position3]
    
    # tilting its head
    Touchpad_position4 = [70,  50, 0, 0, -50, 50, 30, 0, 80, 20, 50, 50, 20,-20]
    Touchpad_position4 = [ x * motion.TO_RAD for x in Touchpad_position4]
    
    # bring back left arm
    Touchpad_position5 = [70,  0, -60, -70, -70, 50, 30, 0, 80, 20, 50, 50, 20,-20]
    Touchpad_position5 = [ x * motion.TO_RAD for x in Touchpad_position5]
    
    pFractionMaxSpeed = 0.3
    pFractionMaxSpeed2 = 0.15
    pFractionMaxSpeed3 = 0.25
    tts.post.say("Please select the one you want on the touch pad")
    
    motionProxy.angleInterpolationWithSpeed(Touchpad, Touchpad_position, pFractionMaxSpeed)
    time.sleep(1.5)
    if above18:
        tts.post.say("We have a choice of Guiness, Corona, Kopperberg and an non alcoholic beer!")
    
        motionProxy.post.angleInterpolationWithSpeed(Touchpad, Touchpad_position2, pFractionMaxSpeed2)
        motionProxy.angleInterpolationWithSpeed(Touchpad, Touchpad_position3, pFractionMaxSpeed2)
        motionProxy.angleInterpolationWithSpeed(Touchpad, Touchpad_position4, pFractionMaxSpeed2)
    
        #tts.post.say(")
    
        motionProxy.angleInterpolationWithSpeed(Touchpad, Touchpad_position3, pFractionMaxSpeed2)
    else:
        tts.post.say("You are under 18,")
        
        motionProxy.post.angleInterpolationWithSpeed(Touchpad, Touchpad_position2, pFractionMaxSpeed2)
        motionProxy.angleInterpolationWithSpeed(Touchpad, Touchpad_position5, pFractionMaxSpeed3)
        
        tts.post.say("but don't worry, we have an non alcoholic beer for you!")
    
    
def dance(tts, motionProxy, postureProxy):
    # Stand to start the dance with the correct balance/posture
    postureProxy.goToPosture("StandInit", 0.5)
    
    tts.post.say("Look at my dance skills!")
    
    time.sleep(2.0)
    # chacha moves
    footStepsList = []

    # 1) Step forward with your left foot
    footStepsList.append([["LLeg"], [[0.06, 0.1, 0.0]]])

    # 2) Sidestep to the left with your left foot
    footStepsList.append([["LLeg"], [[0.00, 0.16, 0.0]]])

    # 3) Move your right foot to your left foot
    footStepsList.append([["RLeg"], [[0.00, -0.1, 0.0]]])

    # 4) Sidestep to the left with your left foot
    footStepsList.append([["LLeg"], [[0.00, 0.16, 0.0]]])

    # 5) Step backward & left with your right foot
    footStepsList.append([["RLeg"], [[-0.04, -0.1, 0.0]]])

    # 6)Step forward & right with your right foot
    footStepsList.append([["RLeg"], [[0.00, -0.16, 0.0]]])

    # 7) Move your left foot to your right foot
    footStepsList.append([["LLeg"], [[0.00, 0.1, 0.0]]])

    # 8) Sidestep to the right with your right foot
    footStepsList.append([["RLeg"], [[0.00, -0.16, 0.0]]])

    ###############################
    # Send Foot step
    ###############################
    stepFrequency = 0.8
    clearExisting = False
    nbStepDance = 2 # defined the number of cycle to make

    for j in range( nbStepDance ):
        for i in range( len(footStepsList) ):
            motionProxy.setFootStepsWithSpeed(
                footStepsList[i][0],
                footStepsList[i][1],
                [stepFrequency],
                clearExisting)



def joke(tts, motionProxy, postureProxy):

    tts.post.say("To beer or not to beer, that is the question.")
    tts.post.say("Do you know who wrote this?")
    time.sleep(1.0)
    tts.post.say("Shake sBeer ")

def joke1(tts, motionProxy, postureProxy):
    
    # Initial Position
    postureProxy.goToPosture("StandInit",1)
    time.sleep(1.0)
    # Joint Matrix
    JokeL = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"]
    JokeR = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
    JokeH = ["HeadYaw", "HeadPitch"]
    # Joke telling successive positions
    Joke_position1L = [45, 30, -90, 0, -90, 50]
    Joke_position1L = [ x * motion.TO_RAD for x in Joke_position1L]
    
    Joke_position2L = [70, -18, -20, -40, -90, -50]
    Joke_position2L = [ x * motion.TO_RAD for x in Joke_position2L]
    
    Joke_position1R = [45, -30, 90, 0, 90, 50]
    Joke_position1R = [ x * motion.TO_RAD for x in Joke_position1R]
    
    Joke_position2R = [70, 18, 20, 40, 90, -50]
    Joke_position2R = [ x * motion.TO_RAD for x in Joke_position2R]
    
    pFractionMaxSpeed = 0.3

    Joke_position1H = [0,-20]
    Joke_position1H = [ x * motion.TO_RAD for x in Joke_position1H]

    Joke_position2H = [0,20]
    Joke_position2H = [ x * motion.TO_RAD for x in Joke_position2H]
    
    tts.post.say("The past,")
    motionProxy.angleInterpolationWithSpeed(JokeL, Joke_position1L, pFractionMaxSpeed)
    
    tts.post.say("the present ")
    motionProxy.angleInterpolationWithSpeed(JokeR, Joke_position1R, pFractionMaxSpeed)
    
    tts.post.say("and the future walk into bar.")
    motionProxy.angleInterpolationWithSpeed(JokeH, Joke_position1H, pFractionMaxSpeed)
    
    time.sleep(1.5)
    tts.post.say("It was tense.")
    pFractionMaxSpeed1 = 0.4
    motionProxy.post.angleInterpolationWithSpeed(JokeL, Joke_position2L, pFractionMaxSpeed1)
    motionProxy.post.angleInterpolationWithSpeed(JokeR, Joke_position2R, pFractionMaxSpeed1)
    motionProxy.post.angleInterpolationWithSpeed(JokeH, Joke_position2H, pFractionMaxSpeed1)

def joke2(tts, motionProxy, postureProxy):
    
    # Joint Matrix
    Joke2L = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"]
    Joke2R = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
    Joke2H = ["HeadYaw", "HeadPitch"]
    
    # Joke telling successive positions
    Joke2_position1L = [90, 40, 0, -90, 50, -50]
    Joke2_position1L = [ x * motion.TO_RAD for x in Joke2_position1L]
    
    Joke2_position2L = [60, 40, -70, -50, -50, 50]
    Joke2_position2L = [ x * motion.TO_RAD for x in Joke2_position2L]
    
    Joke2_position1R = [90, -40, 0, 90, -50, -50]
    Joke2_position1R = [ x * motion.TO_RAD for x in Joke2_position1R]
    
    Joke2_position2R = [10, 0, 0, 0, 0, 50]
    Joke2_position2R = [ x * motion.TO_RAD for x in Joke2_position2R]
    
    Joke2_position3R = [10, 18, 0, 80, 0, 50]
    Joke2_position3R = [ x * motion.TO_RAD for x in Joke2_position3R]
    
    Joke2_position4R = [10, 0, 0, 0, 100, 50]
    Joke2_position4R = [ x * motion.TO_RAD for x in Joke2_position4R]
    
    Joke2_position1H = [0,-20]
    Joke2_position1H = [ x * motion.TO_RAD for x in Joke2_position1H]

    Joke2_position2H = [10,-20]
    Joke2_position2H = [ x * motion.TO_RAD for x in Joke2_position2H]
    
    pFractionMaxSpeed = 0.15
    pFractionMaxSpeed1 = 0.25
    pFractionMaxSpeed2 = 0.4
    
    motionProxy.post.angleInterpolationWithSpeed(Joke2L, Joke2_position1L, pFractionMaxSpeed)
    motionProxy.post.angleInterpolationWithSpeed(Joke2R, Joke2_position1R, pFractionMaxSpeed)
    motionProxy.post.angleInterpolationWithSpeed(Joke2H, Joke2_position1H, pFractionMaxSpeed)
    tts.post.say("A neutron walks into a bar.")
    
    time.sleep(2.0)
    tts.post.say("How much for a drink?, the neutron says,")
    motionProxy.post.angleInterpolationWithSpeed(Joke2L, Joke2_position2L, pFractionMaxSpeed1)
    motionProxy.post.angleInterpolationWithSpeed(Joke2H, Joke2_position2H, pFractionMaxSpeed1)
    
    time.sleep(3.0)
    tts.post.say("The barman answers: For you? no charge")
    motionProxy.post.angleInterpolationWithSpeed(Joke2L, Joke2_position1L, pFractionMaxSpeed1)
    motionProxy.post.angleInterpolationWithSpeed(Joke2R, Joke2_position2R, pFractionMaxSpeed1)
    motionProxy.post.angleInterpolationWithSpeed(Joke2R, Joke2_position3R, pFractionMaxSpeed2)
    motionProxy.post.angleInterpolationWithSpeed(Joke2R, Joke2_position2R, pFractionMaxSpeed1)
    
    time.sleep(3.0)
    tts.post.say("Get it?")
    motionProxy.angleInterpolationWithSpeed(Joke2R, Joke2_position4R, pFractionMaxSpeed1)
    
    
def choice(tts, motionProxy, postureProxy, drink):
    
    sentence = "You have chosen " + drink
    Head = ["HeadYaw", "HeadPitch"]
    
    tts.say(sentence)
    tts.post.say("While you are waiting for your drink, let me entertain you!")
    
    Head_position1 = [0,-20]
    Head_position1 = [ x * motion.TO_RAD for x in Head_position1]
    pFractionMaxSpeed = 0.1
    motionProxy.post.angleInterpolationWithSpeed(Head, Head_position1, pFractionMaxSpeed)

def pickup(tts, motionProxy, postureProxy, drink):
    ID = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand","RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand", "HeadYaw", "HeadPitch"]
    
    sentence = "Your " + drink + " is ready!"
    tts.say(sentence)
    tts.post.say("You can pick it up from the bar!")
    
    # pointing at the ID reader
    ID_position = [30,  -20, 0, -60, 0, 50,70,-50, 0, 0, 50, 50,-50,10]
    ID_position = [ x * motion.TO_RAD for x in ID_position]
    
    pFractionMaxSpeed = 0.3
    
    motionProxy.post.angleInterpolationWithSpeed(ID, ID_position, pFractionMaxSpeed)
    
def main(ip, port):
    
    robotIP = "192.168.43.70"
    PORT = 9559
    above18 = False
    drink = "Corona"
    tts    = ALProxy("ALTextToSpeech", robotIP, PORT)
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    motionProxy.setStiffnesses("Body", 1.0)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

    postureProxy.goToPosture("StandInit",0.5)

    # Wake up robot
    motionProxy.wakeUp()
    #hello_ID(tts, motionProxy,postureProxy)
    
    #drink_select(tts, motionProxy, postureProxy, above18)
    
    #choice(tts, motionProxy, postureProxy, drink)
    pickup(tts, motionProxy, postureProxy, drink)
    #dance(tts, motionProxy, postureProxy)
   
    #joke1(tts, motionProxy, postureProxy)
    
    #joke2(tts, motionProxy, postureProxy)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default= "192.168.43.70",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)
