import numpy as np
import tensorflow as tf

class myAutoencoder(tf.keras.Model):
  def __init__(self, fomSize, romSize=10):
    super(myAutoencoder, self).__init__()
    self.fomSize = fomSize
    self.romSize = romSize

  def build(self, input_shape):
    self.dense1 = tf.keras.layers.Dense(200, input_dim=input_shape, activation=tf.nn.relu)
    self.dense2 = tf.keras.layers.Dense(64, input_dim=200, activation=tf.nn.relu)
    self.dense3 = tf.keras.layers.Dense(self.romSize, input_dim=64, activation=tf.nn.relu)

    self.dense4 = tf.keras.layers.Dense(64, input_dim=self.romSize, activation=tf.nn.relu)
    self.dense5 = tf.keras.layers.Dense(200, input_dim=64, activation=tf.nn.relu)
    self.dense6 = tf.keras.layers.Dense(self.fomSize, input_dim=200, activation=None)

  def encoder(self, x):
    x = self.dense1(x)
    x = self.dense2(x)
    x = self.dense3(x)
    return x

  def decoder(self, x):
    x = self.dense4(x)
    x = self.dense5(x)
    x = self.dense6(x)
    return x

  def call(self, x):
    code = self.encoder(x)
    x = self.decoder(code)
    return x

class MyMapper:
  def __init__(self, autoencoderObj):
    self.model_ = autoencoderObj

    self.numModes_ = autoencoderObj.romSize
    fomSize = autoencoderObj.fomSize
    self.fomState0 = np.zeros(fomSize)
    self.fomState1 = np.zeros(fomSize)
    # attention: the jacobian of the mapping must be column-major oder
    # so that pressio can view it without deep copying it, this enables
    # to keep only one jacobian object around and to call the update
    # method below correctly
    self.jacobian_ = np.zeros((fomSize,self.numModes_), order='F')

  def jacobian(self): return self.jacobian_

  def applyMapping(self, romState, fomState):
    fomState[:] = self.model_.decoder(romState.reshape((1,-1)))[:]

  def applyInverseMapping(self, fomState):
    romState = np.zeros(self.numModes_)
    romState[:] = self.model_.encoder(fomState.reshape((1,-1)))[:]
    return romState

  def updateJacobian(self, romState):
    # finite difference to approximate jacobian of the mapping
    romStateLocal = romState.copy()
    self.applyMapping(romStateLocal,self.fomState0)
    eps = 0.001
    for i in range(self.numModes_):
        romStateLocal[i] += eps
        self.applyMapping(romStateLocal, self.fomState1)
        self.jacobian_[:,i] = (self.fomState1 - self.fomState0) / eps
        romStateLocal[i] -= eps

def trainMapping(snapshots, romSize, epochs):
  fomSize = snapshots.shape[0]
  tf.keras.backend.set_floatx('float64')
  model = myAutoencoder(fomSize, romSize)
  model.compile(loss='mean_squared_error', optimizer = tf.keras.optimizers.Adam())
  autoencoder_train = model.fit(snapshots.T, snapshots.T, batch_size=500,epochs=epochs,verbose=0)

  return MyMapper(model)

