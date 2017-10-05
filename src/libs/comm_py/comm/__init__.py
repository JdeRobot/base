from .communicator import Communicator




def init (config, prefix):
	'''
    Starts JdeRobotComm

    @param config: configuration of client

    @type config: dict
    '''
	return Communicator(config, prefix)




	



