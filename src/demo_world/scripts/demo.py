#!/usr/bin/env python3

import rospy, tf, rospkg, random
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import Quaternion, Pose, Point

# SPAWN_Y = 2.525
SPAWN_Y = 3.775
# SPAWN_Z = 1
SPAWN_Z = 0.75

class CubeSpawner():

	# def __init__(self) -> None:
	# 	self.rospack = rospkg.RosPack()
	# 	self.path = self.rospack.get_path('demo_world')+"/urdf/"
	# 	self.cubes = []
	# 	self.cubes.append(self.path+"red_cube.urdf")
	# 	self.cubes.append(self.path+"green_cube.urdf")
	# 	self.cubes.append(self.path+"blue_cube.urdf")
	# 	self.cubes.append(self.path+"spinner.urdf")
	# 	self.col = 0
	# 	self.screw_num = 0

	# 	self.sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
	# 	self.dm = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
	# 	self.ms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

	def __init__(self) -> None:
		self.rospack = rospkg.RosPack()
		self.path = self.rospack.get_path('demo_world')+"/urdf/spinner/"
		self.cubes = []
		# self.cubes.append(self.path+"blue_cube.urdf")
		# self.cubes.append(self.path+"green_cube.urdf")
		# self.cubes.append(self.path+"red_cude.urdf")
		self.cubes.append(self.path+"bearing.urdf")
		self.cubes.append(self.path+"big_screw.urdf")
		self.cubes.append(self.path+"gears.urdf")
		self.cubes.append(self.path+"screw.urdf")
		self.cubes.append(self.path+"spinner_body.urdf")
		self.col = 0
		self.screw_num = 0

		self.sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
		self.dm = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
		self.ms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

	def checkModel(self):
		res = self.ms("cube", "world")
		return res.success

	def getPosition(self):
		res = self.ms(f"spinner_{self.col-1}", "world")
		return res.pose.position.y

	# def getPosition(self):
	# 	res = self.ms("cube", "world")
	# 	return res.pose.position.z

	def spawnModel(self):
		obj = self.cubes[self.col % len(self.cubes)]
		with open(obj,"r") as f:
			obj_urdf = f.read()
		
		quat = tf.transformations.quaternion_from_euler(0,0,0)
		orient = Quaternion(quat[0],quat[1],quat[2],quat[3])
		pose = Pose(Point(x=0,y=SPAWN_Y,z=SPAWN_Z), orient)
		self.sm(f"spinner_{self.col}", obj_urdf, '', pose, 'world')
		self.col += 1
		# if self.col<2:
		# 	self.col += 1
		# else:
		# 	self.col = 0
		rospy.sleep(1)

	# def spawnModel(self):
	# 	# print(self.col)
	# 	cube = self.cubes[self.col]
	# 	with open(cube,"r") as f:
	# 		cube_urdf = f.read()
		
	# 	quat = tf.transformations.quaternion_from_euler(0,0,0)
	# 	orient = Quaternion(quat[0],quat[1],quat[2],quat[3])
	# 	pose = Pose(Point(x=0,y=-0.55,z=0.75), orient)
	# 	self.sm("cube", cube_urdf, '', pose, 'world')
	# 	if self.col<2:
	# 		self.col += 1
	# 	else:
	# 		self.col = 0
	# 	rospy.sleep(1)

	def deleteModel(self):
		self.dm("cube")
		rospy.sleep(1)

	def deleteModels(self):
		for i in range(self.col):
			self.dm(f"spinner_{i}")
		rospy.sleep(1)

	def shutdown_hook(self):
		self.deleteModels()
		print("Shutting down")

	# def shutdown_hook(self):
	# 	self.deleteModel()
	# 	print("Shutting down")


if __name__ == "__main__":
	print("Waiting for gazebo services...")
	rospy.init_node("spawn_cubes")
	rospy.wait_for_service("/gazebo/delete_model")
	rospy.wait_for_service("/gazebo/spawn_urdf_model")
	rospy.wait_for_service("/gazebo/get_model_state")
	r = rospy.Rate(15)
	cs = CubeSpawner()
	rospy.on_shutdown(cs.shutdown_hook)
	while not rospy.is_shutdown():
		if cs.getPosition() > SPAWN_Y + 0.3 or cs.getPosition() < SPAWN_Y - 0.3:
			cs.spawnModel()

		# if cs.checkModel()==False:
		# 	cs.spawnModel()
		# elif cs.getPosition()<0.05:
		# 	cs.deleteModel()
		r.sleep()
		