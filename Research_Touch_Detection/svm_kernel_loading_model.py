import pickle
import numpy as np

#Having the input set as an array in the format of the library
prediction = np.array([[20000,21200]])

#Loading the desired model and extracting the prediction
filename = 'SVM_linear_kernel.sav'
loaded_model = pickle.load(open(filename,'rb'))
result = loaded_model.predict(prediction)
print(result)

