#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import torch
import transformers
from transformers import AutoTokenizer, AutoModelForCausalLM
#how should we generate the string 
#is printing str is correct 

# Define a Chatbot class: A class that recieves questions from Promter class, 
# answering these question using a language model (LM), the model inference time 
# is greater than the rate of which the ros loop works (1 second as the requirement), 
# so it sends "..." (loading sign) every second till the model finishes the processing.
class Chatbot:
  def __init__(self):
    """
    * Init function of the Chatbot class. It does the following:
    1. Initializes the Chatbot node.
    2. Defines output_str, a variable that carries the output of LM.
    3. Defines publisher and subscriber.
    4. Defines tokenizer and LM.
    5. Calls the publisher function.
    """

    rospy.init_node('Chatbot')
    
    self.output_str = ""

    # init
    self.tokenizer = AutoTokenizer.from_pretrained("togethercomputer/RedPajama-INCITE-Chat-3B-v1")
    self.model = AutoModelForCausalLM.from_pretrained("togethercomputer/RedPajama-INCITE-Chat-3B-v1", torch_dtype=torch.float16, load_in_8bit=True)


    # Declare publisher and subscriber
    self.publisher = rospy.Publisher("/OpenAssistant/PromptAnswers", String, queue_size=10)
    self.subscriber = rospy.Subscriber("/OpenAssistant/PromptQuestions", String, self.subscriber_callback)

    # Call publisher
    self.publish()

  def publish(self):
    """
    * Publish function of the Chatbot class. It does the following:
    1. It defines the rate of which the ros loop works (1 second as the requirement)
    2. It goes  into an infinite loop that does the following:
          a. Checks the output_str variable if it is empty (i.e. the model hasn't
             finished processing yet), it sends "..." (loading sign.)
          b. Otherwise, it sends model response to the prompter.
    """

    rate = rospy.Rate(1)
    while True:
      if(self.output_str == ""):
        self.publisher.publish("...")

      else:
        self.publisher.publish(self.output_str)
        self.output_str = ""
      rate.sleep()

  def subscriber_callback(self, data):
    """
    * Subscriber call back function. It does the following:
    1. If it does not recieve a new question it returns an empty string.
    2. Otherwise it does the following :
      a. Tokenizes the sent question.
      b. Passes the output of the tokenization to the LM.
      c. Decodes the model output and assigns it to the output_str variable so that it
         can be sent to the Prompter.
    """

    if(data.data == ""):
      return
    else:
        prompt = data.data
        inputs = self.tokenizer(prompt, return_tensors='pt').to(self.model.device)
        input_length = inputs.input_ids.shape[1]
        outputs = self.model.generate(
            **inputs, max_new_tokens=15, do_sample=True, temperature=0.8, top_p=0.7, top_k=10, return_dict_in_generate=True
        )
        token = outputs.sequences[0, input_length:]
        self.output_str = self.tokenizer.decode(token)



if __name__ == '__main__':
    try:
        # Define the Chatbot class
        chatbot = Chatbot()
        # Spin Ros Node so that it doesn't shutdown.
        rospy.spin()
    except rospy.ROSInterruptException:
        pass