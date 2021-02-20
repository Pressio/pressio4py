import torch
import torch.optim as optim
import torch.nn.functional as F
import pathlib
import numpy as np

#----------------------------------------
class myAutoencoder(torch.nn.Module):
  def __init__(self, fomSize, romSize=10):
    super(myAutoencoder, self).__init__()
    self.encoder = myEncoder(fomSize, romSize)
    self.decoder = myDecoder(fomSize, romSize)

  def forward(self, x):
    code = self.encoder(x)
    x = self.decoder(code)
    return x, code

  def train(self, dataloader, optimizer, n_epochs, loss=torch.nn.MSELoss()):
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, factor=0.8, min_lr=1e-6)
    for epoch in range(n_epochs):
      total_train_loss = 0.0
      for data,label in dataloader:
        optimizer.zero_grad()
        output, latent = self.forward(data)
        loss_size = loss(output, label)
        loss_size.backward()
        optimizer.step()
      total_train_loss += loss_size.item()
      scheduler.step(total_train_loss)

class myEncoder(torch.nn.Module):
  def __init__(self, fomSize, romSize):
    super(myEncoder, self).__init__()
    self.fc1 = torch.nn.Linear(fomSize, 200)
    self.fc2 = torch.nn.Linear(200, 64)
    self.fc3 = torch.nn.Linear(64, romSize)

  def forward(self, x):
    x = self.fc1(x)
    x = F.elu(x)
    x = self.fc2(x)
    x = F.elu(x)
    x = self.fc3(x)
    x = F.elu(x)
    return x

class myDecoder(torch.nn.Module):
  def __init__(self, fomSize, romSize):
    super(myDecoder, self).__init__()
    self.romSize_ = romSize
    self.fomSize_ = fomSize
    self.fc1 = torch.nn.Linear(romSize, 64)
    self.fc2 = torch.nn.Linear(64, 200)
    self.fc3 = torch.nn.Linear(200, fomSize)

  def forward(self, x):
    x = self.fc1(x)
    x = F.elu(x)
    x = self.fc2(x)
    x = F.elu(x)
    x = self.fc3(x)
    return x

class MyMapper:
  def __init__(self, decoderObj, encoderObj):
    self.decoder_ = decoderObj
    self.encoder_ = encoderObj

    self.numModes_ = decoderObj.romSize_
    fomSize = decoderObj.fomSize_
    self.fomState0 = np.zeros(fomSize)
    self.fomState1 = np.zeros(fomSize)
    # attention: the jacobian of the mapping must be column-major oder
    # so that pressio can view it without deep copying it, this enables
    # to keep only one jacobian object around and to call the update
    # method below correctly
    self.jacobian_ = np.zeros((fomSize,self.numModes_), order='F')

  def jacobian(self): return self.jacobian_

  def applyMapping(self, romState, fomState):
    fomState[:] = self.decoder_(torch.Tensor(romState)).detach().numpy()

  def applyInverseMapping(self, fomState):
    romState = np.zeros(self.numModes_)
    romState[:] = self.encoder_(torch.Tensor(fomState)).detach()[:]
    return romState

  def updateJacobian(self, romState):
    self.updateJacobianFD(romState)

  def updateJacobianFD(self, romState):
    # finite difference to approximate jacobian of the mapping
    romStateLocal = romState.copy()
    self.applyMapping(romStateLocal,self.fomState0)
    eps = 0.001
    for i in range(self.numModes_):
        romStateLocal[i] += eps
        self.applyMapping(romStateLocal, self.fomState1)
        self.jacobian_[:,i] = (self.fomState1 - self.fomState0) / eps
        romStateLocal[i] -= eps

  def updateJacobianExact(self, romState):
    # use pytorch autodifferentiation to compute jacobian of the mapping
    # slower than finite difference currently
    J = torch.autograd.functional.jacobian(self.decoder_, torch.Tensor(romState))
    self.jacobian_[:,:] = J.detach()[:,:]

#----------------------------------------
def trainMapping(snapshots, romSize, epochs, enable_restart=False):
  fomSize = snapshots.shape[0]
  model = myAutoencoder(fomSize, romSize)
  optimizer = optim.AdamW(model.parameters(), lr=5e-3)

  if enable_restart:
    if pathlib.Path('TrainingCheckpoint.tar').is_file():
      print("Loading checkpoint")
      checkpoint = torch.load('TrainingCheckpoint.tar')
      model.load_state_dict(checkpoint['model_state_dict'])
      optimizer.load_state_dict(checkpoint['optimizer_state_dict'])

  samples = torch.utils.data.TensorDataset(torch.Tensor(snapshots.T), torch.Tensor(snapshots.T))
  loader = torch.utils.data.DataLoader(samples, batch_size=500, shuffle=True)
  model.train(loader, optimizer, n_epochs=epochs)

  if enable_restart:
    torch.save({
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict()
               },
               'TrainingCheckpoint.tar')

  return MyMapper(model.decoder, model.encoder) 