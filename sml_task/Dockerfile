# Obtain the ros neotic image.
FROM osrf/ros:noetic-desktop-full

# Update the system and install pip 
RUN apt-get update && apt-get install -y \
    python3-pip


# Install huggingface transformers, datasets, and other libraries to speedup the inference of the ML model.
RUN pip3 install transformers datasets sacremoses sentencepiece accelerate bitsandbytes

# Upgrade the sytem and install git. 
RUN apt-get upgrade -y && \
    apt-get install -y git

# Make the workspace
RUN mkdir -p catkin_ws/src

# Clear the cashe and clone the repository. 
#The Personal Access Token (PAT) was added to the start of the URL as it's a private link.
ARG CACHEBUST=1 
RUN git clone https://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@github.com/frankjjiang/baraa_interview

# Copy the sml_task ros package to the workspace src directory.
RUN cp -r baraa_interview/sml_task catkin_ws/src/

# Make sure that the start_sml is executable.
RUN  chmod u+x /catkin_ws/src/sml_task/start_sml.sh 

# Make the start_sml shell file the entry point of the container (execute it directly.)
ENTRYPOINT ["/bin/bash", "/catkin_ws/src/sml_task/start_sml.sh"]
