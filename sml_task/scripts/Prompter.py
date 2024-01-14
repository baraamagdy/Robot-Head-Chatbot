#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import time
import rospy

# Define class Prompter. A class that loops through 10 predefined Strings and send them 
# to the chatbot node. The Promter class sends a string each second as follows:
#       1. In the beginning, it keeps sending the first message till it gets a response
#          from the chat bot
#       2. Once it sends the first message, it waits for the chatbot to respond.
#          Whilte waiting, it sends empty strings to the chatbot and writes the response
#          of the empty string to the console (the reponse is the loading sign "...") 
#       3. Once it gets the first response from the chatbot, it prints the response and 
#          sends the next prompt and wait as explained in point 2.
#       4. The node keeps repeating step 2 & 3 untill it is shutdown

class Promter():
  def __init__(self):
    """
    * Init function of the Prompter class. It does the following:
    1. Initializes the node Prompter.
    2. Defines the last recieved string and the hardcoded messages and messages_idx.
    3. Defines the publisher and subscriber.
    4. Calls the publish function.
    """
    rospy.init_node('Prompter')
    self.last_str = "..."

    # Define messages and message_idx
    self.message_idx = 0
    self.messages = ["<human>: Who is Alan Turing?\n<bot>:",
                "<human>: what's your name? and what's your job ?\n<bot>:",
                "<human>: what's 2 + 2 ?\n<bot>:",
                "<human>: what's the capital of Sweden ?\n<bot>:",
                "<human>: what's planning in robotics ?\n<bot>:",
                "<human>: what does ROS stand for in robtoics ?\n<bot>:",
                "<human>: what is model predictive control ?\n<bot>:",
                "<human>: Was gradient descent really first applied for rockets' aiming ?\n<bot>:",
                "<human>: What is docker ?\n<bot>:",
                "<human>: Goodbye!! \n<bot>:"]

    # Declare publisher and subscriber
    self.publisher = rospy.Publisher("/OpenAssistant/PromptQuestions", String, queue_size=10)
    self.subscriber = rospy.Subscriber("/OpenAssistant/PromptAnswers", String, self.subscriber_callback)

    # Call publish function.
    self.publish()

  def publish(self):
    """
    * Publish function of the Prompter class. It does the following:
    1. It defines the rate of which the ros loop works (1 second as the requirement)
    2. It goes  into an infinite loop that does the following:
          a. If this is the first message to be sent to the subscriber, it keeps sending 
             it till it gets a response.
          b. Otherwise, it sends an empty string which denotes that it doesn't request 
             a prompt from the chatbot (this is done while it's waiting for a response),
             and also fullfilling the requirement of publish eaching second. 
    """
    # Define the rate at which the loop operates (1 second)
    rate = rospy.Rate(1)
    self.first_time = True
    
    while True:
      # If this is the first time or if it's not waiting for a response, send a new message.
      if(self.last_str != "..." or self.first_time):
        self.publisher.publish(self.messages[self.message_idx])
        if(not self.first_time):
            self.message_idx = (self.message_idx + 1) % 10
      # Otherwise, send an empty string while waiting for a response.
      else:
        self.publisher.publish("")
      rate.sleep()

  def subscriber_callback(self, data):
    """
    * Subscriber call back function. It does the following:
    1. If the chatbot is still processing the answer (denoted by "..."), print it to
       the user.
    2. Otherwise (the chatbot prepared an answer):
         a. Print the response of the chatbot.
         b. Overwrite the last_str variable denoting that it has recieved a response 
            allowing the publish function to send a new prompt (message)
         c. Turns off the first time flag.
        
    """
    if(data.data == "..."):
      print(data.data, end="\r")
      self.last_str = data.data
    else:
      print(data.data)
      self.last_str = data.data
      self.first_time = False

if __name__ == '__main__':
    try:
        # Defnie the prompter class
        prompter = Promter()
        # Spin Ros Node so that it doesn't shutdown.
        rospy.spin()
    except rospy.ROSInterruptException:
        pass