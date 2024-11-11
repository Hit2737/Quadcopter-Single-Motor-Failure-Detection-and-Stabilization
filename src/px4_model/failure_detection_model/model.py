import tensorflow as tf
from tensorflow.keras.layers import Conv1D, BatchNormalization, Activation, Dropout, GRU, Dense

class MotorFailureDetectionModel(tf.keras.Model):
    def __init__(self, input_shape):
        super(MotorFailureDetectionModel, self).__init__()
        
        # Step 1: CONV layer
        self.conv1d = Conv1D(filters=196, kernel_size=15, strides=4, input_shape=input_shape)
        self.batch_norm1 = BatchNormalization()
        self.relu = Activation("relu")
        self.dropout1 = Dropout(rate=0.8)
        
        # Step 2: First GRU Layer
        self.gru1 = GRU(units=128, return_sequences=True)
        self.dropout2 = Dropout(rate=0.8)
        self.batch_norm2 = BatchNormalization()
        
        # Step 3: Second GRU Layer
        self.gru2 = GRU(units=128, return_sequences=True)
        self.dropout3 = Dropout(rate=0.8)
        self.batch_norm3 = BatchNormalization()
        self.dropout4 = Dropout(rate=0.8)
        
        # Step 4: Dense output layer
        self.output_layer = Dense(5, activation="sigmoid")
        
    def call(self, inputs):
        # Step 1: CONV layer
        x = self.conv1d(inputs)
        x = self.batch_norm1(x)
        x = self.relu(x)
        x = self.dropout1(x)
        
        # Step 2: First GRU Layer
        x = self.gru1(x)
        x = self.dropout2(x)
        x = self.batch_norm2(x)
        
        # Step 3: Second GRU Layer
        x = self.gru2(x)
        x = self.dropout3(x)
        x = self.batch_norm3(x)
        x = self.dropout4(x)
        
        # Step 4: Dense output layer
        x = self.output_layer(x)
        
        return x