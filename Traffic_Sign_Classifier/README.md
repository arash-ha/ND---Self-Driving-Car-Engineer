# **Traffic Sign Recognition** 

The aim of this project was to accurately classify traffic signs from the [German Traffic Signs dataset](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset) by means of a convolutional neural network.

**Build a Traffic Sign Recognition Network**

Here are the steps I followed:
* Load the dataset
* Explore, summarize and visualize the dataset
* Pre-process, balance and augment the dataset
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images

You can view my workflow in Traffic_Signs_Classifier.ipynb.


[//]: # (Image References)

[image1]: ./images/hist_orig.png "Visualisation"
[image2]: ./images/hist_new.png "New Visualisation"
[image3]: ./images/processed_images.png "Processed Images"
[image4]: ./images/test_web.png "Web Test"
[image5]: ./images/results_web1.png "Web Results 1"
[image6]: ./images/results_web2.png "Web Results 2"
[image7]: ./images/results_web3.png "Web Results 3"
[image8]: ./images/results_web4.png "Web Results 4"
[image9]: ./images/results_web5.png "Web Results 5"
[image10]: ./images/results_web6.png "Web Results 6"


### Data Set Summary & Exploration
signs data set:

* The size of training set is 34799
* The size of the validation set is 4410
* The size of test set is 12630
* The shape of a traffic sign image is (32, 32, 3)
* The number of unique classes/labels in the data set is 43

####  Exploratory Visualization of the Dataset

The German Traffic Signs dataset can be seen in the below histogram. It is split into 43 categories of signs, with the number of images in each category on the y-axis. 
Since the distribution of the dataset is unequal, this can cause bias into the training model. Besides, ~200 number of examples for training some classes is simply too low, and it will need to be augmented.


![alt text][image1]

#### Data Preparation and Pre-processing

The steps for the process of the data to be able to train are in the following:

#### 1. Grayscaling
Grayscaling decreases the image from 3 RGB channels to a single layer, and it would be beneficial to us in terms of reducing the number of variables the network has to deal with. This improves processing times, however, loss of information because of color could is a major concern. Besides, testing with both RGB and BW images show no significant difference in performance.

#### 2. Equalization
In order to improve contrast and to provide clearer, equalization method can be used. This helps to define edges better, as well. First, OpenCV's histogram equalization was used, but it seems that the results are blurry and poor contrast. Then, Skimage's adaptive CLAHE was employed and it took longer to process the data but the results are better compared to equalization.

#### 3. Normalization
Normalization is used to scale the intensity of the image from (0, 255) to (-1, 1). 

#### 4. Augmentation (Transformation)
Due to the low number of examples for some classes, rebalancing the dataset can be utilized in order to prevent bias in the mode. Therefore, the size of th dataset is multiplied by three in all classes, including the ones already heavily represented. 

#### 5. Shuffling
The shuffling of the dataset is important step to prevent the traing of the model relies on the order of the datasets instead of the features. 

#### Grayscaling, Equalisation and Normalization
Normalization is used to scale the intensity of the image from (0, 255) to (-1, 1). 

```
from skimage import exposure
from sklearn.utils import shuffle
from skimage import exposure
import cv2

def Preprocess(X):
    X_norm = []  
    for image in X:
            gry = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            equ = exposure.equalize_adapthist(gry)
            equ = (equ * 2.0/equ.max())
            equ = np.reshape(equ,(32,32,1))-1
            X_norm.append(equ)
    return np.array(X_norm)
```

#### Augmentation (Transformation)
Rotation within a certain angle and wrapping through projective transformation(skimage) are used. The reason for choosing the projective transforms is the similarity os change in camera perspective.

This link [here](http://navoshta.com/) helps me a lot to understand what I should do in this part of the project.

```
from skimage.transform import ProjectiveTransform
from skimage.transform import rotate
from skimage.transform import warp

def randomTransform(image, intensity):
    
    # Rotate image within a set range, amplified by intensity of overall transform.
    rotation = 20 * intensity
    # print(image.shape)
    rotated = rotate(image, np.random.uniform(-rotation,rotation), mode = 'edge')
    
    # Projection transform on image, amplified by intensity.
    image_size = image.shape[0]
    magnitude = image_size * 0.3 * intensity
    tl_top = np.random.uniform(-magnitude, magnitude)     # Top left corner, top margin
    tl_left = np.random.uniform(-magnitude, magnitude)    # Top left corner, left margin
    bl_bottom = np.random.uniform(-magnitude, magnitude)  # Bottom left corner, bottom margin
    bl_left = np.random.uniform(-magnitude, magnitude)    # Bottom left corner, left margin
    tr_top = np.random.uniform(-magnitude, magnitude)     # Top right corner, top margin
    tr_right = np.random.uniform(-magnitude, magnitude)   # Top right corner, right margin
    br_bottom = np.random.uniform(-magnitude, magnitude)  # Bottom right corner, bottom margin
    br_right = np.random.uniform(-magnitude, magnitude)   # Bottom right corner, right margin
    
    transform = ProjectiveTransform()
    transform.estimate(np.array((
            (tl_left, tl_top),
            (bl_left, image_size - bl_bottom),
            (image_size - br_right, image_size - br_bottom),
            (image_size - tr_right, tr_top))),
            np.array((
            (0, 0),
            (0, image_size),
            (image_size, image_size),
            (image_size, 0)
            )))
    transformed = warp(rotated, transform, output_shape = (image_size, image_size), order = 1, mode = 'edge')
    return transformed

def batchAugment(X, y, multiplier = 2):
    X_train_aug = []
    y_train_aug = []
    for i in range(len(X)):
        for j in range(multiplier):
            augmented = randomTransform(X[i], 0.5)
            X_train_aug.append(augmented)
            y_train_aug.append(y[i])
        X_train_aug.append(X[i])
        y_train_aug.append(y[i])
        
    X_train_aug, y_train_aug = shuffle(X_train_aug, y_train_aug)
    print("New augmented size is: ", len(X_train_aug))
    return X_train_aug, y_train_aug
```

#### Mini pipeline for the preprocessing
Mini pipeline of the pre-processing phase.
```
# Processing both training and validation sets
X_train_proc = batchPreprocess(X_train)
X_valid_proc = batchPreprocess(X_valid)

print("Pre-processing complete!")
```
Rebalancing based on representation of classes.
```
unique, counts = np.unique(y_train, return_counts=True)
print("Original distribution of classes: ", counts)
multiplier = [int(round(max(counts)/i)) for i in counts] # Required multiplier for each class augmentation.
print("Multipliers for each class: ", multiplier)
multiplier = [i-2 for i in multiplier]

X_train_aug = X_train_p
y_train_aug = y_train

for i in types:
    if multiplier[i] > 0: # Ignoring classes It is not needed for oversampling
        X_train_add = []
        y_train_add = []
        index = np.where(y_train==i)
        for j in index:
            X_train_add.append(X_train_p[j])
            y_train_add.append(y_train[j])
        X_train_add = np.array(X_train_add)
        X_train_add = np.reshape(X_train_add, (len(index[0]),32,32,1))
        y_train_add = np.array(y_train_add)
        y_train_add = np.reshape(y_train_add, (len(index[0])))
    
        print("Class: ", i+1)
        X_train_add, y_train_add = batchAugment(X_train_add, y_train_add, multiplier[i])
        X_train_aug = np.vstack((X_train_aug, X_train_add))
        print("New total dataset size: ",len(X_train_aug))
        y_train_aug = np.append(y_train_aug, y_train_add)
        print("")


unique, counts = np.unique(y_train_aug, return_counts=True)
print("New distribution of classes: ", counts)
```
Output:
```
Original distribution of classes:  [ 180 1980 2010 1260 1770 1650  360 1290 1260 1320 1800 1170 1890 1920  690
  540  360  990 1080  180  300  270  330  450  240 1350  540  210  480  240
  390  690  210  599  360 1080  330  180 1860  270  300  210  210]

New distribution of classes:  [1980 1980 2010 1260 1770 1650 2160 1290 1260 1320 1800 1170 1890 1920 2070
 2160 2160  990 1080 1980 2100 1890 1980 1800 1920 1350 2160 2100 1920 1920
 1950 2070 2100 1797 2160 1080 1980 1980 1860 1890 2100 2100 2100]
```

Here's a histogram of the new distribution. Not perfect, but vastly improved from the earlier spread.

![alt text][image2]


Lastly, I tripled the dataset size with rotation and transforms.
```python
# Triple the dataset size by rotation and transformation
X_train_aug, y_train_aug = batchAugment(X_train_aug, y_train_aug, 1)    

print("Augmentation complete!")
```
Output:
```
Augmentation complete!
```

#### Visualisation of the Pre-processing Steps
In the following image, it can be observed the image before, during and after processing! 
It can be seen that the edges are better defined after equalisation, and the variations produced by the augmentation step.


![alt text][image3]

### Model Architecture and Testing
In this step, the LeNet-5 architecture is implemented.

My final model is constructed as follows:

| Layer         		|     Description	        					| 
|:---------------------:|:---------------------------------------------:| 
| Input         		| 32x32x1 Grayscale image   							| 
| Convolution 5x5     	| 1x1 stride, valid padding, output 28x28x6 	|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  output 14x14x6 				|
| Convolution 5x5	    | 1x1 stride, valid padding, output 10x10x16      									|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  output 5x5x16 				|
| Fully connected		| Input 400, output 120        									|
| RELU					|												|
| Dropout					|		0.85 rate										|
| Fully connected		| Input 120, output 84        									|
| RELU					|												|
| Dropout					|		0.85 rate										|
| Fully connected		| Input 84, output 43     									|
| Softmax				| With tf.reduce_mean loss and AdamOptimizer        									|
 
#### Parameters and Training Pipline
Parameters are chosen after trial and error, and observation of model behaviour. To train the model, the AdamOptimizer is used.
~~~
EPOCHS = 20
BATCH_SIZE = 128
rate = 0.001

logits = LeNet(x)
cross_entropy = tf.nn.softmax_cross_entropy_with_logits(labels=one_hot_y, logits=logits)
loss_operation = tf.reduce_mean(cross_entropy)
optimizer = tf.train.AdamOptimizer(learning_rate = rate)
training_operation = optimizer.minimize(loss_operation)
~~~

#### Results

1. At the first step, grayscaling and OpenCV equalization are applied to the dataset. the classifier achieved 92% accuracy on validating dataset.
2. Then, the augmentation was doubled without rebalancing. In this case, the dataset is large but it is strongly biased. The model achieved 95% on the validation dataset.
3. After rebalancing and 3x augmentation, the validation accuracy the model achieved is 96%.

#### Test Set Pulled From The Web
Here are six German traffic signs that I found on the web. 
![alt text][image4]

### Results
```
Test Accuracy = 0.33
```

![alt text][image5]
![alt text][image6]
![alt text][image7]
![alt text][image8]
![alt text][image9]
![alt text][image10]


It seems that the model only predict Bumpy road and Slippery road correctly. The reason could be clarity of these sign compared to others.
Some steps can be taken in order to increase the performance:
1. more training on shadowed or obstructed signs
2. working on batchAugment function

### Conclusion
Overall, the model performed rather well in initial testing, but faltered on images taken from the web. The reason is that the images from the web are cmpletely different from the supplied training set. Further refinement, augmentation and perhaps simply more, varied data could help the model generalise and deal with the challenge better.

Implementing models from other successful papers can be employed to gain a better intuition of the design choices when building a model. I notify that the LeNet model was initially built for low-resolution classification of the MNIST dataset, a considerably simpler and less varied challenge.

