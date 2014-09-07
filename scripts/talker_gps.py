#!/usr/bin/python
# Written by Dan Mandle http://dan.mandle.me September 2012
# License: GPL 2.0
 
import os
import rospy
import std_msgs.msg
import sensor_msgs.msg
from gps import *
from time import *
import time
import threading
 
 
class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    #self.gpsd #bring it in scope
    self.gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true
 
  def run(self):
    while not rospy.is_shutdown():
      self.gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer

def talker():
  pub = rospy.Publisher('gps', sensor_msgs.msg.NavSatFix, queue_size=10) 
  rospy.init_node('talker_gps', anonymous=True)
  r = rospy.Rate(1) # 1hz
  #global gpsp
  print "Starting GPS..."
  gpsp = GpsPoller() # create the thread
  try:
    gpsp.start() #start it up
    print "GPS Started."
    while not rospy.is_shutdown():
      gpsmsg = sensor_msgs.msg.NavSatFix()
      stat = sensor_msgs.msg.NavSatStatus()
      gpsmsg.header = std_msgs.msg.Header()
      gpsmsg.header.frame_id="quad_test"
      gpsmsg.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work

      gpsmsg.latitude = gpsp.gpsd.fix.latitude
      gpsmsg.longitude = gpsp.gpsd.fix.longitude
      gpsmsg.altitude = gpsp.gpsd.fix.altitude
      gpsmsg.position_covariance_type = sensor_msgs.msg.NavSatFix.COVARIANCE_TYPE_KNOWN
      #TODO: need co-variance matrix from data
      stat.status = sensor_msgs.msg.NavSatStatus.STATUS_FIX
      stat.service = sensor_msgs.msg.NavSatStatus.SERVICE_GPS

      gpsmsg.status = stat
      pub.publish(gpsmsg)
      r.sleep()
  except:
    print "unexpected interuption"
    gpsp.running = False
    gpsp.join()




if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException: pass

def example():
  gpsp = GpsPoller() # create the thread
  try:
    gpsp.start() # start it up
    while True:
      #It may take a second or two to get good data
      #print gpsd.fix.latitude,', ',gpsd.fix.longitude,'  Time: ',gpsd.utc
 
      os.system('clear')
 
      print
      print ' GPS reading'
      print '----------------------------------------'
      print 'latitude    ' , gpsd.fix.latitude
      print 'longitude   ' , gpsd.fix.longitude
      print 'time utc    ' , gpsd.utc,' + ', gpsd.fix.time
      print 'altitude (m)' , gpsd.fix.altitude
      print 'eps         ' , gpsd.fix.eps
      print 'err lon     ' , gpsd.fix.epx
      print 'err lat     ' , gpsd.fix.epy
      print 'err vert    ' , gpsd.fix.epv
      print 'err time    ' , gpsd.fix.ept
      print 'err dir     ' , gpsd.fix.epd
      print 'err vel     ' , gpsd.fix.eps
      print 'speed (m/s) ' , gpsd.fix.speed
      print 'climb (m/s) ' , gpsd.fix.climb
      print 'track corse ' , gpsd.fix.track
      print 'mode        ' , gpsd.fix.mode
      print ''
      print 'sats        ' , gpsd.satellites
      print 'sats #      ' , len(gpsd.satellites)
      print ''
      time.sleep(1) #set to whatever
 
  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print "\nKilling Thread..."
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing
  print "Done.\nExiting."

