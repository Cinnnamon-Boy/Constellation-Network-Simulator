from env import CreateEnviroment
port = 5559
seed = 33
startSim = True
train = False #if this parameter is set as Ture, it will train the models by gradient descent, otherwise it will use trained model directly
offLine = False #if this parameter is set as Ture, it will gather experience of agents firstly.
env = CreateEnviroment(portID=port, startSim=startSim, simSeed=seed,train=train,offLine=offLine)
env.simulation()

