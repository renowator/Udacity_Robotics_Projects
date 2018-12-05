[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

## Deep Learning Project ##

In this project, you will train a deep neural network to identify and track a target in simulation. So-called “follow me” applications like this are key to many fields of robotics and the very same techniques you apply here could be extended to scenarios like advanced cruise control in autonomous vehicles or human-robot collaboration in industry.

[image_0]: ./docs/misc/sim_screenshot.png
![alt text][image_0] 

## Setup Instructions
**Clone the repository**
```
$ git clone https://github.com/udacity/RoboND-DeepLearning.git
```

**Download the data**

Save the following three files into the data folder of the cloned repository. 

[Training Data](https://s3-us-west-1.amazonaws.com/udacity-robotics/Deep+Learning+Data/Lab/train.zip) 

[Validation Data](https://s3-us-west-1.amazonaws.com/udacity-robotics/Deep+Learning+Data/Lab/validation.zip)

[Sample Evaluation Data](https://s3-us-west-1.amazonaws.com/udacity-robotics/Deep+Learning+Data/Project/sample_evaluation_data.zip)

**Download the QuadSim binary**

To interface your neural net with the QuadSim simulator, you must use a version QuadSim that has been custom tailored for this project. The previous version that you might have used for the Controls lab will not work.

The simulator binary can be downloaded [here](https://github.com/udacity/RoboND-DeepLearning/releases/latest)

**Install Dependencies**

You'll need Python 3 and Jupyter Notebooks installed to do this project.  The best way to get setup with these if you are not already is to use Anaconda following along with the [RoboND-Python-Starterkit](https://github.com/udacity/RoboND-Python-StarterKit).

If for some reason you choose not to use Anaconda, you must install the following frameworks and packages on your system:
* Python 3.x
* Tensorflow 1.2.1
* NumPy 1.11
* SciPy 0.17.0
* eventlet 
* Flask
* h5py
* PIL
* python-socketio
* scikit-image
* transforms3d
* PyQt4/Pyqt5

## Implement the Segmentation Network
1. Download the training dataset from above and extract to the project `data` directory.
2. Implement your solution in model_training.ipynb
3. Train the network locally, or on [AWS](https://classroom.udacity.com/nanodegrees/nd209/parts/09664d24-bdec-4e64-897a-d0f55e177f09/modules/cac27683-d5f4-40b4-82ce-d708de8f5373/lessons/197a058e-44f6-47df-8229-0ce633e0a2d0/concepts/27c73209-5d7b-4284-8315-c0e07a7cd87f?contentVersion=1.0.0&contentLocale=en-us).
4. Continue to experiment with the training data and network until you attain the score you desire.
5. Once you are comfortable with performance on the training dataset, see how it performs in live simulation!

## Additional data ##
 
 Additional 600 images were added to the training set. These images were taking following the hero manually and most of them contain hero either close-up or in the distance
 
## Network Architecture ##
 
 ** Network Model **
 
 The network consists of 4 encoder and 4 decoder layers (defult kernel_size=7) with additional 1x1 convolution in the middle as well as a convolution with (kernel_size=5, strides = 1) before the final layer
 
 ** Encoder Layer **
 
 Consists of a separable convolution with kernel_size=7 and strides=2 decreasing the image size by 2x2
 Each consecutive encoder layer doubles the amount of filters applied - 16 for layer1 to 128 for layer4
 
 ** Decoder Layer ** 
 
 Takes working layer and a pass through layer that is 2x2 larger 
 Performs bilinear upsampling on the working layer and concatenates it with the pass through layer
 After that performs separable convolution on a pass through layer with (kernel_size=3, strides=1) with no downsampling
 Concatenates these two layers.
 Resulting output_layer is 2x2 larger than the working layer and the same size as pass through layer.
 
 
 ** Hyperparameters **
 
 Learning Rate: 0.01 - Higher learning rate causes unexpected val_loss increases, while lower ones make the model improve loss too slowly
 Batch size: 32 - with p2.xlarge instances the amount of layers causes the system to occasionally run out of resources with batch size 64.
 Num epochs: 20 - I figured out that large amount of epochs make network overfit to the training data spiking (and possibly validation data) the val loss and the final result.
 Steps per epoch: 149 - amount of training data/batch size
 
 ** Changes in the model **
 
 1. Change in optimizer to Nadam which is better with my learning rate.
 2. Add ModelCheckpoint callback to record weights with best val_loss
 
## Result ##
 
 After experimenting with hyperparameters and layers the following setup produced needed final_result of 0.41
 The network configuration is saved at: /data/weights/amazing_aws_weights.h5
 To run the network with these weights load from model_training.ipynb
 [image_1]: ./docs/misc/training_curves.png
 ![alt text][image_1] 
 
 ** Changing training classes **
 
 A change in training class will require to tune in layers and hyperparameters. Smaller objects may need a higher downsampling/upsampling size than the hero.
 
## Scoring ##

To score the network on the Follow Me task, two types of error are measured. First the intersection over the union for the pixelwise classifications is computed for the target channel. 

In addition to this we determine whether the network detected the target person or not. If more then 3 pixels have probability greater then 0.5 of being the target person then this counts as the network guessing the target is in the image. 

We determine whether the target is actually in the image by whether there are more then 3 pixels containing the target in the label mask. 

Using the above the number of detection true_positives, false positives, false negatives are counted. 

**How the Final score is Calculated**

The final score is the pixelwise `average_IoU*(n_true_positive/(n_true_positive+n_false_positive+n_false_negative))` on data similar to that provided in sample_evaulation_data

## Experimentation: Testing in Simulation
1. Copy your saved model to the weights directory `data/weights`.
2. Launch the simulator, select "Spawn People", and then click the "Follow Me" button.
3. Run the realtime follower script
```
$ python follower.py my_amazing_model.h5
```

**Note:** If you'd like to see an overlay of the detected region on each camera frame from the drone, simply pass the `--pred_viz` parameter to `follower.py`
