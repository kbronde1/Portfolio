# Explanation of file format

All assignments were required to be submitted as both a Jupyter notebook (.ipynb) and report (.pdf). I have attached my assignments only in the report format, omitting odd-numbered homework assignments as they were theory-based and therefore contain no coding. A brief description of the even-numbered homeworks can be found below.

### Homework 2
#### Autograd Structure
In this assignment, I implemented my own AudoGrad structure. I then implemented a linear classifier using said AutoGrad class, trained it on a 2-dimensional toy dataset, and displayed my results. To quantify the model’s performance, I passed its output through a softmax function and then use Cross Entropy loss to measure agreement with the target output.

I also wrote a Python program based on my AutoGrad structure that iteratively computes a small step in the direction of negative gradient. Lastly, I compared the performance of single-layer and multi-layer classifiers on a different dataset, again using my AutoGrad structure.

## Homework 4
#### MLP via PyTorch
For this assignment, I manually implemented an "AND gate" and an "OR gate" for decision boundary predictions. I then built a Multi-Layer Perceptron using sigmoid activation functions to replace the “AND/OR” gates. I trained my data with mini-batch support and implemented visualization functions for my results. I followed the given model hyperparameters, and adjusted the depth and width to increase the MLP's capacity as expected later on.
#### CNNs via PyTorch
This assignment also had me create a Convolutional Neural Network to use for classification of the Fashion MNIST dataset.

# Homework 6
#### Unsupervised Pre-Training
In the first part of this assignment, I attempted the 2017 Endoscopic Instrument Challenge. I was given a pre-processed dataset consisting of endoscopic frame images (not in sequential order). The goal was to train a network which takes each RGB frame as an input and predicts a pixel-wise segmentation mask that labels the target instrument type and background tissue. Additionally, I introduced an unsupervised pre-training method and compared the performance of training on a small labeled dataset with/without pre-training. This is relevant for real-life medical image problems, where there is usually a shortage of data labels.
#### Transfer Learning
I tested the accuracies of an untrained and a pre-trained VGG16 model on classification of the Fashion MNIST dataset.
