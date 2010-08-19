#!/usr/bin/python

import roslib; roslib.load_manifest('simmechanics_to_urdf')
import rospy
import sys
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_matrix
import xml.dom.minidom
from xml.dom.minidom import Document
import threading
import math
import numpy
import yaml

# Conversion Factors
INCH2METER = 0.0254
SLUG2KG = 14.5939029
SLUGININ2KGMM = .009415402
MM2M = .001

# Special Reference Frame(s)
WORLD = "WORLD"

# Arbitrary List of colors to give pieces different looks
COLORS =[("green", "0 1 0 1"), ("black", "0 0 0 1"), ("red", "1 0 0 1"), 
	 ("blue", "0 0 1 1"), ("yellow", "1 1 0 1"), ("pink", "1 0 1 1"), 
	 ("cyan", "0 1 1 1"), ("green", "0 1 0 1"), ("white", "1 1 1 1"), 
	 ("dblue", "0 0 .8 1"), ("dgreen", ".1 .8 .1 1"), ("gray", ".5 .5 .5 1")]

class Converter:
	def __init__(self):
		# initialize member variables
		self.links = {}
		self.frames = {}
		self.joints = {}
		self.names = {}
		self.colormap = {}
		self.colorindex = 0
		self.usedcolors = {}

		# Start the Transform Manager
		self.tfman = TransformManager()
		self.tfman.start()

		# Extra Transforms for Debugging
		self.tfman.add([0,0,0], [0.70682518,0,0,0.70682518], "ROOT", WORLD) # rotate so Z axis is up

	def convert(self, filename, configfile, mode):
		self.mode = mode

		# Parse the configuration file
		self.parseConfig(configfile)

		# Parse the input file
		self.parse(xml.dom.minidom.parse(filename))
		self.buildTree(self.root)

		# Create the output 
		self.result = Document()
		self.output(self.root)

		# output the output
		if mode == "xml":
			print self.result.toprettyxml()
		if mode == "graph":
			print self.graph()
		if mode == "groups":
			print self.groups(root)

	def parseConfig(self, configFile):
		"""Parse the Configuration File, if it exists.
		   Set the fields the default if the config does 
		   not set them """
		if configFile == None:
			configuration = {}
		else:
			configuration = yaml.load(file(configFile, 'r'))
			if configuration == None:
				configuration = {}

		self.freezeList = []
		self.redefinedjoints = {}

		self.root = configuration.get('root', None)
		self.extrajoints = configuration.get('extrajoints', {})
		self.filenameformat = configuration.get('filenameformat', "%s")
		self.forcelowercase = configuration.get('forcelowercase', False)
		self.scale = configuration.get('scale', None)
		self.freezeAll = configuration.get('freezeAll', False)
		self.baseframe = configuration.get('baseframe', WORLD)

		# Get lists converted to strings
		self.removeList = [ str(e) for e in configuration.get('remove', []) ]
		self.freezeList = [ str(e) for e in configuration.get('freeze', []) ]

		# Get map with key converted to strings
		jointmap = configuration.get('redefinedjoints', {})
		for x in jointmap.keys():
			self.redefinedjoints[str(x)] = jointmap[x]

		# Add Extra Frames
		for frame in configuration.get('moreframes', []):
			self.tfman.add(frame['offset'], frame['orientation'], frame['parent'], frame['child'])
		

	def parse(self, element):
		"""Recursively goes through all XML elements 
		   and branches off for important elements"""
		name = element.localName
		# Grab name from root element AND recursively parse
		if name == "PhysicalModelingXMLFile":
			dict = getDictionary(element)
			self.name = dict['name']

		if name == "Body":
			self.parseLink(element)
		elif name == "SimpleJoint":
			self.parseJoint(element)
		elif name == "Ground":
			dict = getDictionary(element)
			self.parseFrames(dict['frame'], "GROUND")
		else: 
			for child in element.childNodes:
				self.parse(child)

	def parseLink(self, link):
		"""Parse the important bits of a link element"""
		linkdict = getDictionary(link)
		uid = self.getName(linkdict['name'])
		linkdict['neighbors'] = []
		linkdict['children'] = []
		linkdict['jointmap'] = {}

		# Save the frames for separate parsing
		frames = linkdict['frames']
		linkdict['frames'] = None

		# Save the color if it exists
		if 'MaterialProp' in linkdict:
			colorelement = linkdict['MaterialProp'][1]
			color = colorelement.childNodes[0].data
			linkdict['MaterialProp'] = None
			linkdict['color'] = color.replace(",", " ") + " 1"
			

		self.links[uid] = linkdict
		self.parseFrames(frames, uid)

		# Save First Actual Element as Root, if not defined already
		if self.root == None and "geometryFileName" in linkdict:
			self.root = uid

	def parseFrames(self, frames, parent):
		"""Parse the frames from xml"""
		for frame in frames:
			if frame.nodeType is frame.TEXT_NODE:
				continue
			fdict = getDictionary(frame)
			fid = str(frame.getAttribute("ref"))
			fdict['parent'] = parent

			offset = getlist(fdict['position'])
			units = fdict['positionUnits']
			for i in range(0, len(offset)):
				offset[i] = convert(offset[i], units)

			orientation = getlist(fdict['orientation'])
			quat = matrixToQuaternion(orientation)
			
			# If the frame does not have a reference number,
			# use the name plus a suffix (for CG or CS1...
			# otherwise ignore the frame
			if fid == "":
				name = fdict['name']
				if name == "CG":
					fid = parent + "CG"
				elif name == "CS1":
					fid = parent + "CS1"
				else:
					continue

			self.tfman.add(offset, quat, WORLD, fid)
			self.frames[fid] = fdict

	def parseJoint(self, element):
		"""Parse the joint from xml"""
		dict = getDictionary(element)
		joint = {}
		joint['name'] = dict['name']
		uid = self.getName(joint['name'])

		frames = element.getElementsByTagName("Frame")
		joint['parent'] = str(frames[0].getAttribute("ref"))
		joint['child'] = str(frames[1].getAttribute("ref"))
		type = element.getElementsByTagName("Primitive")

		# If there multiple elements, assume a fixed joint
		if len(type)==1:
			pdict = getDictionary(type[0])
			joint['type'] = pdict['name']
			joint['axis'] = pdict['axis']
			if joint['type'] == 'weld':
				joint['type'] = 'fixed'
		else:
			joint['type'] = 'fixed'

		# Ignore joints on the remove list
		if joint['parent'] in self.removeList:
			return

		# Force joints to be fixed on the freezeList
		if joint['parent'] in self.freezeList or self.freezeAll:
			joint['type'] = 'fixed'

		# Redefine specified joints on redefined list
		if joint['parent'] in self.redefinedjoints.keys():
			jdict = self.redefinedjoints[joint['parent']]
			if 'name' in jdict:
				uid = jdict['name']

			# Default to continuous joints
			joint['type'] = jdict.get('type', 'continuous')

			if 'axis' in jdict:
				joint['axis'] = jdict['axis']
			if 'limits' in jdict:
				joint['limits'] = jdict['limits']

		self.joints[uid] = joint

	def buildTree(self, root):
		"""Reduce the graph structure of links and joints to a tree
		   by breadth first search. Then construct new coordinate frames
		   from new tree structure"""
	
		# Create a list of all neighboring links at each link
		for jid in self.joints:
			jointdict = self.joints[jid]
			if "Root" in jointdict['name']:
				continue
			pid = self.getLinkNameByFrame(jointdict['parent'])
			cid = self.getLinkNameByFrame(jointdict['child'])
			parent = self.links[pid]
			child = self.links[cid]

			parent['neighbors'].append(cid)
			parent['jointmap'][cid] = jid
			child['neighbors'].append(pid)
			child['jointmap'][pid] = jid

		# Add necessary information for any user-defined joints
		for (name, extrajoint) in self.extrajoints.items():
			pid = extrajoint['pid']
			cid = extrajoint['cid']
			jorigin = extrajoint['jorigin']
			newframe = name + "_frame"

			self.links[pid]['neighbors'].append(cid)
			self.links[pid]['jointmap'][cid] = name
			self.links[cid]['neighbors'].append(pid)
			self.links[cid]['jointmap'][pid] = name
			self.joints[name] = {'name': name, 'parent': jorigin, 'child': newframe}
			for (k,v) in extrajoint['attributes'].items():
				self.joints[name][k] = v
			self.frames[jorigin] = {'parent': pid}
			self.frames[newframe] = {'parent': cid}

		# Starting with designated root node, perform BFS to 
		# create the tree
		queue = [ root ]
		self.links[root]['parent'] = "GROUND"
		while len(queue) > 0:
			id = queue.pop(0)
			link = self.links[id]
			for n in link['neighbors']:
				nbor = self.links[n]
				# if a neighbor has not been visited yet, 
				# add it as a child node
				if not 'parent' in nbor:
					nbor['parent'] = id
					queue.append(n)
					link['children'].append(n) 

		# build new coordinate frames
		for id in self.links:
			link = self.links[id]
			if not 'parent' in link:
				continue
			parentid = link['parent']
			if parentid == "GROUND":
				ref = self.baseframe
			else:
				joint = self.joints[link['jointmap'][parentid]]
				ref = joint['parent']
			# The root of each link is the offset to the joint
			# and the rotation of the CS1 frame
			(off1, rot1) = self.tfman.get(WORLD, ref)
			(off2, rot2) = self.tfman.get(WORLD, id + "CS1")
			self.tfman.add(off1, rot2, WORLD, "X" + id)
			

	def output(self, rootid):
		"""Creates the URDF from the parsed document.
		   Makes the document and starts the recursive build process"""
		doc = self.result;
		self.root = doc.createElement("robot")
		doc.appendChild(self.root)
		self.root.setAttribute("name", self.name)
		self.outputLink(rootid)
		self.processLink(rootid)

	def processLink(self, id):
		""" Creates the output for the specified node's
		    child links, the connecting joints, then 
		    recursively processes each child """
		link = self.links[id]
		for cid in link['children']:
			jid = link['jointmap'][cid]
						
			self.outputLink(cid)
			self.outputJoint(jid, id)
			self.processLink(cid)

	def outputLink(self, id):
		""" Creates the URDF output for a single link """
		doc = self.result
		linkdict = self.links[id]		 
		if linkdict['name'] == "RootPart":
			return
		
		link = doc.createElement("link")
		link.setAttribute("name", id)

		visual = doc.createElement("visual")
		inertial = doc.createElement("inertial")
		collision = doc.createElement("collision")
		link.appendChild(visual)
		link.appendChild(inertial)
		link.appendChild(collision)

		# Define Geometry
		geometry = doc.createElement("geometry")
		mesh = doc.createElement("mesh")
		filename = linkdict['geometryFileName']
		if self.forcelowercase:
			filename = filename.lower()
		
		mesh.setAttribute("filename", self.filenameformat % filename)
		if self.scale != None:
			mesh.setAttribute("scale", self.scale)
		geometry.appendChild(mesh)
		visual.appendChild(geometry)
		collision.appendChild(geometry.cloneNode(1))

		# Define Inertial Frame
		mass = doc.createElement("mass")
		units = linkdict['massUnits']
		massval = convert(float(linkdict['mass']), units)
		mass.setAttribute("value", str(massval))
		inertial.appendChild(mass)

		inertia = doc.createElement("inertia")
		matrix = getlist(linkdict["inertia"])
		units = linkdict['inertiaUnits']
	
		for i in range(0,len(matrix)):	
			matrix[i] = convert(matrix[i], units)
	
		inertia.setAttribute("ixx", str(matrix[0]))
		inertia.setAttribute("ixy", str(matrix[1]))
		inertia.setAttribute("ixz", str(matrix[2]))
		inertia.setAttribute("iyy", str(matrix[4]))
		inertia.setAttribute("iyz", str(matrix[5]))
		inertia.setAttribute("izz", str(matrix[8]))
		inertial.appendChild(inertia)

		# Inertial origin is the center of gravity
		iorigin = doc.createElement("origin")
		(off, rot) = self.tfman.get("X" + id, id+"CG")
		rpy = list(euler_from_quaternion(rot))
		iorigin.setAttribute("xyz", " ".join(map(str,zero(off))))
		iorigin.setAttribute("rpy", " ".join(map(str,zero(rpy))))
		inertial.appendChild(iorigin)

		# Create link's origin
		origin = doc.createElement("origin")
		
		# Visual offset is difference between origin and CS1
		(off, rot) = self.tfman.get("X" + id, id+"CS1")
		rpy = list(euler_from_quaternion(rot))
		origin.setAttribute("xyz", " ".join(map(str,zero(off))))
		origin.setAttribute("rpy", " ".join(map(str,zero(rpy))))
		visual.appendChild(origin)
		collision.appendChild(origin.cloneNode(1))

		# Define Material
		material = doc.createElement("material")
		
		# Use specified color, if exists. Otherwise, get random color
		if 'color' in linkdict:
			cname = "%s_color"%id
			rgba = linkdict['color']
		else:
			(cname, rgba) = self.getColor(linkdict['name'])
		material.setAttribute("name", cname)
		visual.appendChild(material)
		
		# If color has already been output, only output name
		if not cname in self.usedcolors:
			color = doc.createElement("color")
			color.setAttribute("rgba", rgba)
			material.appendChild(color)
			self.usedcolors[cname] = True
				
		self.root.appendChild(link)

	def getColor(self, s):
		""" Gets a two element list containing a color name,
		    and it's rgba. The color selected is based on the mesh name.
		    If already seen, returns the saved color
		    Otherwise, returns the next available color"""
		if s in self.colormap:
			return self.colormap[s]
		color = COLORS[self.colorindex]
		self.colormap[s] = color
		self.colorindex = (self.colorindex + 1) % len(COLORS)
		return color
		
	def outputJoint(self, id, parentname):
		""" Outputs URDF for a single joint """
		doc = self.result
		jointdict = self.joints[id]

		if "Root" in jointdict['name']:
			return

		joint = doc.createElement("joint")		
		joint.setAttribute('name', id)

		# Define the parent and child
		pid = self.getLinkNameByFrame(jointdict['parent'])
		cid = self.getLinkNameByFrame(jointdict['child'])

		# If the original joint was reversed while building the tree,
		# swap the two ids
		if parentname != pid:
			cid = pid
			pid = parentname
		
		parent = doc.createElement("parent")
		child = doc.createElement("child")
		parent.setAttribute('link', pid)
		child.setAttribute('link', cid)
		joint.appendChild(parent)
		joint.appendChild(child)

		# Define joint type
		jtype = jointdict['type']
		joint.setAttribute('type', jtype)
		if 'limits' in jointdict:
			limit = doc.createElement("limit")
			for (k,v) in jointdict['limits'].items():
				limit.setAttribute(str(k), str(v))
			joint.appendChild(limit)
		if 'axis' in jointdict and jtype != 'fixed':
			axis = doc.createElement("axis")
			xyz = jointdict['axis'].replace(',', ' ')
			axis.setAttribute('xyz', xyz)
			joint.appendChild(axis)
	
		# Define the origin
		origin = doc.createElement("origin")
		(off, rot) = self.tfman.get("X" + pid, "X" + cid)
		rpy = list(euler_from_quaternion(rot))
		off = zero(off)
		rpy = zero(rpy)
		origin.setAttribute("xyz", " ".join(map(str,off)))
		origin.setAttribute("rpy", " ".join(map(str,rpy)))
		joint.appendChild(origin)

		self.root.appendChild(joint)

	def getName(self, basename):
		"""Return a unique name of the format 
		   basenameD where D is the lowest number
		   to make the name unique"""
		index = 1
		name = basename + str(index)
		while name in self.names:
			index = index + 1
			name = basename + str(index)
		self.names[name] = 1
		return name

	def getLinkNameByFrame(self, key):
		"""Gets the link name from the frame object"""
		return self.frames[key]['parent']

	def graph(self):
		"""For debugging purposes, output a graphviz 
		   representation of the tree structure, noting
		   which joints have been reversed and which have 
		   been removed"""
		graph = "digraph proe {\n"
		for jkey in self.joints:
			joint = self.joints[jkey]
			pref = joint['parent']
			cref = joint['child']
			label = pref + ":" + cref
			pkey = self.getLinkNameByFrame(pref)
			ckey = self.getLinkNameByFrame(cref)
			case = 'std'
			if pkey != "GROUND":
				parent = self.links[pkey]
				if not ckey in parent['children']:
					child = self.links[ckey]
					if pkey in child['children']:
						case = 'rev'
					else:
						case = 'not'
			pkey = pkey.replace("-", "_")
			ckey = ckey.replace("-", "_")

			if (case == 'std' or case == 'rev') and (joint['type'] != "fixed"):
				style = " penwidth=\"5\""
			else:
				style = "";

			if case == 'std':
				s = pkey + " -> " + ckey + " [ label = \""+label+"\""; 
			elif case == 'not':
				s = pkey + " -> " + ckey + " [ label = \""+label+"\" color=\"yellow\""
			elif case == 'rev':
				s = ckey + " -> " + pkey + " [ label = \""+label+"\" color=\"blue\""
			s = s + style + "];"
			
			if not "Root" in s and "-> SCR_" not in s:
				graph = graph + s + "\n"
		return graph + "}\n"

	def groups(self, root):
		""" For planning purposes, print out lists of 
                    all the links between the different joints"""
		self.groups = {}
		self.makeGroup(root, "BASE")
		s = ""
		for key in self.groups.keys():
			s = s + key + ":\n\t"
			ids = self.groups[key]
			for id in ids:
				s = s+id + " "
			s = s + "\n\n"
		return s

	def makeGroup(self, id, gid):
		""" Helper function for recursively gathering
		    groups of joints. """
		if gid in self.groups:
			idlist = self.groups[gid]
			idlist.append(id)
		else:
			idlist = [id]
		self.groups[gid] = idlist
		link = self.links[id]
		for child in link['children']:
			jid = link['jointmap'][child]
			joint = self.joints[jid]
			if joint['type'] == 'weld':
				ngid = gid
			else:
				ngid = jid
			
			self.makeGroup(child, ngid)

def getDictionary(tag):
	"""Builds a dictionary from the specified xml tag
	   where each child of the tag is entered into the dictionary
	   with the name of the child tag as the key, and the contents
	   as the value. Also removes quotes from quoted values"""
	x = {}
	for child in tag.childNodes:
		if child.nodeType is child.TEXT_NODE:
			continue
		key = str(child.localName)
		if len(child.childNodes) == 1:
			data = str(child.childNodes[0].data)
			if data[0] == '"' and data[-1] == '"':
				if len(data) != 2:
					x[key] = data[1:-1]
			else:
				x[key] = data
		else:
			data = child.childNodes
			x[key] = data
	return x

def getlist(string):
	"""Splits a string of comma delimited floats to
	   a list of floats"""
	slist = string.split(",")
	flist = []
	for s in slist:
		flist.append(float(s))
	return flist


def convert(value, units):
	"""Convert value from the specified units to mks units"""
	if units == 'kg' or units == 'm' or units == 'kg*m^2':
		return value
	elif units == 'slug*in^2':
		return value * SLUGININ2KGMM
	elif units == 'slug':
		return value * SLUG2KG
	elif units == 'in':
		return value * INCH2METER
	elif units == 'mm':
		return value * MM2M
	else:
		raise Exception("unsupported mass unit: %s" % units)

def matrixToQuaternion(matrix):
	"""Concert 3x3 rotation matrix into a quaternion"""
	(R11, R12, R13, R21, R22, R23, R31, R32, R33) = matrix
	# Build 4x4 matrix
	M = [[R11, R21, R31, 0],
	     [R12, R22, R32, 0],
	     [R13, R23, R33, 0],
	     [0,   0,   0,   1]]
	A = numpy.array(M)
	[w,x,y,z] = quaternion_from_matrix(A) 
	return [w,x,y,z]

def quaternion_to_rpy(quat):
	"""Convert quaternion into roll pitch yaw list (in degrees)"""
	rpy = list(euler_from_quaternion(quat))
	for i in range(0, len(rpy)):
		rpy[i] = rpy[i]*180/math.pi
	return rpy

def zero(arr):
	"""Converts any numbers less than 1e-7 to 0 in the array"""
	for i in range(0,len(arr)):
		if math.fabs(arr[i]) < 1e-7:
			arr[i] = 0
	return arr


class TransformManager(threading.Thread):
	"""Bridge to the ROS tf package. Constantly broadcasts
	   all tfs, and retrieves them on demand."""

	def __init__(self):
		threading.Thread.__init__(self)
		self.running = True
		self.tfs = []
		self.listener = tf.TransformListener()

	def run(self):
		"""Broadcasts tfs until shutdown"""
		rate = rospy.Rate(10.0)
		br = tf.TransformBroadcaster()
		while self.running:
			for transform in self.tfs:
				br.sendTransform(transform['trans'], 
					transform['rot'],
					rospy.Time.now(),
					transform['child'],
					transform['parent'])
			rate.sleep()

	def add(self, trans, rot, parent, child):
		"""Adds a new transform to the set"""
		tf = {'trans':trans, 'rot':rot, 'child':child, 'parent':parent}
		self.tfs.append(tf)

	def get(self, parent, child):
		"""Attempts to retrieve a transform"""
		attempts = 0
		rate = rospy.Rate(10.0)
		while attempts < 50:
			try:
				(off,rpy) = self.listener.lookupTransform(parent, child, rospy.Time(0))
				return [list(off), list(rpy)]
			except (tf.LookupException, tf.ConnectivityException):
				attempts+=1
				rate.sleep()
				continue

		raise Exception("Attempt to transform %s exceeded attempt limit" % (parent + "/" + child))

	def printTransform(self, parent, child):
		"""Attempts to print the specified transform"""
		(off, rot) = self.get(parent, child)
		rpy = quaternion_to_rpy(rot)

		s = parent + "-->" + child
		l = (30 - len(s))*" "
		print "%s%s[%+.5f %+.5f %+.5f] \t [%+3.3f %+.3f %+.3f] \t [%+.6f %+.6f %+.6f %+.6f] " \
			% (s, l, off[0], off[1], off[2], rpy[0], rpy[1], rpy[2], rot[0], rot[1], rot[2], rot[3])

	def kill(self):
		"""Stops thread from running"""
		self.running = False

if __name__ == '__main__':
	argc = len(sys.argv)
	if argc == 3:
		filename = sys.argv[1]
		config = None
		mode = sys.argv[2]
	elif argc == 4:
		filename = sys.argv[1]
		config = sys.argv[2]
		mode = sys.argv[3]
	else:
		print "Usage: " + sys.argv[0] + " {XML filename} [configfile] {tf|xml|graph|groups|none}"
		sys.exit(-1)
	try:
		rospy.init_node('convert') 
		con = Converter()
		try:
			con.convert(filename, config, mode)
			while mode == "tf" and not rospy.is_shutdown():
				None
		except Exception:
			raise
		finally:
			con.tfman.kill()
			
	except rospy.ROSInterruptException: pass

