class ErrorObj(object):
	def __init__(self, InitValue=0):
		self.__act = InitValue
		self.old = InitValue

	@property
	def act(self):
		return self.__act

	@act.setter
	def act(self, value):
		self.old = self.__act
		self.__act = value


class ErrorVector():
	def __init__(self):
		self.x = ErrorObj()
		self.y = ErrorObj()
		self.z = ErrorObj()


error = ErrorVector()

for i in range(10):
	print(error.x.act, error.x.old)
	error.x.act=error.x.act+1

