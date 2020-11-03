import pandas as pd
import numpy as np
import pickle
import matplotlib.pyplot as plt
from sklearn.cross_validation import train_test_split
from sklearn.svm import SVC
from sklearn.metrics import classification_report, confusion_matrix

#Loading the dataset
data = pd.read_csv("touch_detection_data.csv")

#Putting the features into x and the classification into y
x = data.drop('Class', axis=1)
y = data['Class']

#Spliting the dataset into train and test dataset
x_train, x_test, y_train, y_test = train_test_split(x, y, test_size = 0.2)

#Linear Kernel (Best fit for single channels)
#Conducting the fitting for linear kernel
svclassifier_linear = SVC(kernel = 'linear', verbose=True)
svclassifier_linear.fit(x_train, y_train)

#Saving the model
filename = 'SVM_linear_kernel.sav'
pickle.dump(svclassifier_linear,open(filename,'wb'))

#Obtaining results using the test dataset
y_pred_linear = svclassifier_linear.predict(x_test)

print('Linear Kernel:')
print(confusion_matrix(y_test, y_pred_linear))
print(classification_report(y_test, y_pred_linear))


'''
#Polynomial Kernel
svclassifier_poly = SVC(kernel = 'poly', degree = 8)
svclassifier_poly.fit(x_train,y_train)

y_pred_poly = svclassifier_poly.predict(x_test)

print('Polynomial Kernel:')
print(confusion_matrix(y_test, y_pred_poly))
print(classification_report(y_test, y_pred_poly))

#Gaussian Kernel
svclassifier_G = SVC(kernel = 'rbf')
svclassifier_G.fit(x_train, y_train)

y_pred_G = svclassifier_G.predict(x_test)

print('Gaussian Kernel:')
print(confusion_matrix(y_test, y_pred_G))
print(classification_report(y_test, y_pred_G))


#Sigmoid
svclassifier_sigmoid = SVC(kernel = 'sigmoid')
svclassifier_sigmoid.fit(x_train, y_train)

y_pred_sigmoid = svclassifier_sigmoid.predict(x_test)

print('Sigmoid:')
print(confusion_matrix(y_test, y_pred_sigmoid))
print(classification_report(y_test, y_pred_sigmoid))
'''


