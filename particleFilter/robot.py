import random
import numpy as np
import matplotlib.pyplot as plt

def display(robot,room,particles=[]):
  '''
  Displays the locations of the particles as black dots and the true robot
  position as a larger red circle within the room
  '''
  plt.ion()
  plt.clf()
  rpose=robot.pose()
  pxs=[]
  pys=[]
  for particle in particles:
    x,y,yaw=particle.pose()
    pxs.append(x)
    pys.append(y)
  plt.plot(pxs,pys,'k.')
  plt.plot([rpose[0]],[rpose[1]],'ro')
  plt.xlim([0,room.maxX])
  plt.ylim([0,room.maxY])
  plt.show(block=False)

def fixAngle(angle):
  '''
  Returns an equivalent angle which is between 0 and 2*pi
  '''
  while angle<0:
    angle+=2*np.pi
  while angle>=2*np.pi:
    angle-=2*np.pi
  return angle
  
def dist(x1,y1,x2,y2):
  '''
  Returns the Euclidean distance between two points
  '''
  return np.sqrt(pow(x1-x2,2)+pow(y1-y2,2))

class Room:
  '''
  A class which keeps track of the room dimensions, and will calculate the true
  distance from a pose to a wall
  '''
  def __init__(self,maxX=100,maxY=50):
    '''
    By default, the room is 100m wide and 50m tall
    '''
    self.maxX=maxX
    self.maxY=maxY

  def trueObservation(self,x,y,yaw):
    '''
    Given an x, y, and yaw, will calculate the true distance (no noise) from
    that point, at that orientation, to the wall of the room.

    There are probably faster ways of doing this, but it works.
    '''
    m=np.tan(yaw)
    b=y-m*x
    points=[]
    points.append((0,b))
    points.append((self.maxX,m*self.maxX+b))
    y3=0
    x3=(y3-b)/m
    points.append((x3,y3))
    y4=self.maxY
    x4=(y4-b)/m
    points.append((x4,y4))

    infront=None
    if yaw<(np.pi/2):
      test=lambda pt : pt[0]>x and pt[1]>y
    elif yaw<(np.pi):
      test=lambda pt : pt[0]<x and pt[1]>y
    elif yaw<(3*np.pi/2):
      test=lambda pt : pt[0]<x and pt[1]<y
    else:
      test=lambda pt : pt[0]>x and pt[1]<y
    for pt in points:
      if test(pt) and 0<=pt[0]<=self.maxX and 0<=pt[1]<=self.maxY:
        infront=pt
        break
    if infront is None:
      print(x,y,yaw,points)
    return dist(x,y,infront[0],infront[1])

class Robot:
  '''
  Controls the noisy robot simulation, including driving, turning, and sensing
  distance straight out from the robot.  Straight line driving, turning, and
  observations all follow linear-Gaussian error models.
  '''
  def __init__(self,room,
      x=None,y=None,yaw=None,
      c_o=1,sigma_o=.5,
      b_l=1,sigma_l=.2,
      b_r=1,sigma_r=.1):
    '''
    Must be given a pointer to the room object.  If x, y, and yaw are not
    given, the robot is placed randomly within the room.  Linear-Gaussian
    noise parameters can also be input.  *_o are parameters for observation
    noise, *_l are parameters for straight line driving, and *_r are for
    turning (in radians).
    '''
    
    self.__room=room
    self.__maxX=room.maxX
    self.__maxY=room.maxY

    if x is None:
      self.__x=random.random()*self.__maxX
    else:
      self.__x=x
    if y is None:
      self.__y=random.random()*self.__maxY
    else:
      self.__y=y
    if yaw is None:
      self.__yaw=random.random()*2*np.pi
    else:
      self.__yaw=yaw
    self.__yaw=fixAngle(self.__yaw)


    self.c_o=c_o
    self.sigma_o=sigma_o
    self.b_l=b_l
    self.sigma_l=sigma_l
    self.b_r=b_r
    self.sigma_r=sigma_r


  def drive(self,linearDist):
    '''
    Drives the robot about linearDist meters straight ahead, unless contact
    with a wall causes it to stop short. This action is noisy, as determined by
    the linear-Gaussian parameters b_l and sigma_l input in the object
    initializer.
    '''
    realDist=self.b_l*linearDist+np.random.normal(scale=self.sigma_l)
    toWall=self.__room.trueObservation(self.__x,self.__y,self.__yaw)
    if toWall<realDist:
      realDist=toWall-1e-5
    self.__x+=realDist*np.cos(self.__yaw)
    self.__y+=realDist*np.sin(self.__yaw)

  def turn(self,angle):
    '''
    Turns the robot about angle radians.  This action is noisy, as determined
    by the linear-Gaussian parameters b_r and sigma_r input in the object
    initializer.
    '''
    realAngle=self.b_r*angle+np.random.normal(scale=self.sigma_r)
    self.__yaw+=realAngle
    self.__yaw=fixAngle(self.__yaw)

  def obs(self):
    '''
    Returns three observations of the distance from the robot to the wall.  The
    first is straight out from the robot's nose, minus pi/6.  The second is
    straight out fromt he robot.  The third is straight out, plus pi/6. These
    observations are noisy, as determined by the linear-Gaussian parameters c_o
    and sigma_o input in the object initializer.
    '''
    angles=[fixAngle(angle) for angle in [self.__yaw-np.pi/6,self.__yaw,self.__yaw+np.pi/6]]
    trueObs=[self.__room.trueObservation(self.__x,self.__y,angle) for angle in angles]
    return [self.c_o*ob+np.random.normal(scale=self.sigma_o) for ob in trueObs]

  def pose(self):
    '''
    Returns the true pose of the robot as a tuple, (x,y,yaw).  Of course, using
    this would be cheating, outside of debugging or visualizing the scene.
    '''
    return (self.__x,self.__y,self.__yaw)

if __name__ == "__main__":
  room=Room()
  r=Robot(room,x=1,y=49,yaw=-np.pi/8)
  print(r.pose())
  display(r,room)
  r.drive(5)
  print(r.pose())
  display(r,room)
  r.turn(np.pi/2)
  print(r.pose())
  display(r,room)
  print(r.obs())
