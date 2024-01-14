
# Robot Head Chatbot project

Welcome to the README file for the Robot Head Chatbot project! This project involves the development of a robot head that engages in conversations with users. The project is implemented using the Robot Operating System (ROS) framework, utilizing two ROS nodes: 'Prompter' and 'Chatbot'. The 'Prompter' node handles text input from users, while the 'Chatbot' node employs a Large Language Model (LLM) to generate responses and chat with users in a friendly way.

## Project Overview

The Robot Head Chatbot project aims to create a human-like interaction experience by enabling the robot head to understand and respond to user inputs through spoken language. The project is divided into two primary components: speech recognition and text-to-speech synthesis.

### Nodes
**Prompter Node**: This node is responsible for receiving text input from users. It takes the text from a speech recognition node (to be implemented in a future work) and pases the generated text to the 'Chatbot' node for processing.

**Chatbot Node**: The 'Chatbot' node utilizes a Large Language Model (LLM), similar to ChatGPT, to generate responses based on the input received from the 'Prompter' node. The LLM is tailored to balance accuracy and performance to meet the real-time requirements of the robot head's conversation capabilities.

### Integration with ROS

ROS (Robot Operating System) is a flexible framework for writing robot software. The project leverages ROS for communication between nodes, allowing seamless interaction and data exchange within the system. Future integration with speech recognition and text-to-speech synthesis modules will complete the full conversational loop.

#### Getting Started
To set up and run the Robot Head Chatbot project, follow these steps:
**Prerequisites**: Make sure you have docker installed on a linux machine system. If not, follow the installation instructions on the [docker website](https://docs.docker.com/engine/install/ubuntu/).

1. Clone the Repository: Clone this repository to your local machine.
2. Build Docker Image: Navigate to the root directory of the cloned repository and build the Docker image using the provided Dockerfile: 
`sudo docker build ./ -t IMAGE_TAG`
3. Run Docker Container: Launch the Docker container using the following command:
`sudo docker run -it --name IMAGE_NAME IMAGE_TAG`
4. Conversation Process: Inside the Docker container, the project's entry point is a shell script (`start_sml.sh`) that builds the catkin_ws and runs both ROS nodes ('Prompter' and 'Chatbot'). You can observe the conversation capabilities of the robot head within the container environment.
 
Alternatively, to run the project directlty on your linux based machine (both docker and ros should be installed):
1. Clone the Repository: Clone this repository to your `~/catkin_ws/src/` directory.
2. Build the workspace: run `catkin_make` in the workspace `~/catkin_ws`
3. Source the workspace: run `source devel/setup.bash` in `~/catkin_ws`.
4. Run the project using the pre-made launch file: ROS Launnch: run "roslaunch sml_task sml_launch.launch"

## Future Enhancements
1. Integrate Speech Recognition and Text-to-Speech: Implement and integrate speech recognition and text-to-speech modules with the 'Prompter' and 'Chatbot' nodes, respectively. Update the Dockerfile or shell script to include these modules in the communication pipeline.
2. Multi-Modal Interaction: Integrate additional modes of interaction, such as gesture recognition and facial expression analysis, to create a richer communication experience.
3. Command Awareness: Add commands classification models to tell the robot that it needs to perform an action from a set of predefined actions.

Thank you for your interest in the Robot Head Chatbot project!


