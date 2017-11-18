#!/usr/bin/env python

import rospy
import time

class Initializer :

  def __init__(self) :

    self.launchers_dir = rospy.get_param('~launchers_dir')
    if self.launchers_dir[len(self.launchers_dir) - 1] != '/':
      self.launchers_dir = self.launchers_dir + '/'
     
    
  def create_camera_launch(self) :
    cameras = rospy.get_param('~cameras')
    checkerboard = rospy.get_param('~checkerboard')
    rosRate = rospy.get_param('~rosRate')

    #self.camera_list = []
    for camera in cameras:
      #self.camera_list = self.camera_list + [item]
      session_id = rospy.Time.now().secs

      print 'debug'
      print camera['id']

      ############# write: launch/camera_xx.launch ##############
      file_name = self.launchers_dir + 'camera_' + camera['id'] + '.launch'

      file = open(file_name, 'w')
      file.write('<?xml version="1.0"?>\n')
      file.write('<!-- SESSION ID: ' + str(session_id) + ' -->\n')
      file.write('<launch>\n\n')

      file.write('  <!-- Checkerboard parameters -->\n')
      file.write('  <arg name="rows"          default="' + str(checkerboard['rows']) + '" />\n')
      file.write('  <arg name="cols"          default="' + str(checkerboard['cols']) + '" />\n')
      file.write('  <arg name="cell_width"    default="' + str(checkerboard['cell_width']) + '" />\n')
      file.write('  <arg name="cell_height"   default="' + str(checkerboard['cell_height']) + '" />\n\n')
      
      file.write('  <!-- Camera parameters -->\n')
      file.write('  <arg name="camera_name"     default="' + camera['id'] + '" />\n')

      #if camera['type'] == 'ip': # for ip camera  
      #  if camera['ip'] == '':
      #    file.close()
      #    rospy.logerr('Missing "ip" field for the ip camera!')
      #  file.write('  <arg name="camera_ip"   default="' + camera['ip'] + '" />\n\n')
      if camera['type'] == 'usb': # for ip camera  
        if camera['device'] == '':
          file.close()
          rospy.logerr('Missing "device" field for the usb camera!')
        file.write('  <arg name="camera_device"   default="' + str(camera['device']) + '" />\n\n')
      file.write('  <arg name="intrinsic"   default="$(find ros_visual_localization)/conf/' + camera['intrinsic'] + '" />\n\n') 
      file.write('  <arg name="rosRate"   default="' + str(rosRate) + '" />\n\n') 

      file.write('  <!-- Launching camera -->\n')
      if camera['type'] == 'usb':
        file.write('  <node pkg="ros_visual_localization" type="visual_localization" name="' + camera['id'] + '" output="screen">\n\n')
        file.write('    <param name="camera_device"       value="${arg camera_device}" />\n')
        file.write('    <param name="camera_type"           value="usb" />\n')

      #if camera['type'] == 'ip':
      #  file.write('  <node pkg="ros_visual_localization" type="ipCamera" name="${arg camera_name}" output="screen">\n\n')
      #  file.write('    <param name="${arg camera_name} + "/ip"         value="' + str(camera.ip) + '" />\n')
      file.write('    <param name="camera_name"           value="${arg camera_name}" />\n')
      file.write('    <param name="intrinsic"             value="${arg intrinsic}" />\n')

      file.write('    <param name="rows"                  value="$(arg rows)" />\n')
      file.write('    <param name="cols"                  value="$(arg cols)" />\n')
      file.write('    <param name="cell_width"            value="$(arg cell_width)" />\n')
      file.write('    <param name="cell_height"           value="$(arg cell_height)" />\n\n')
      file.write('    <param name="rosRate"           value="$(arg rosRate)" />\n\n')

      file.write('    <rosparam command="load"            file="$(arg intrinsic" />\n\n')


      file.write('  </node>\n\n')
      file.write('</launch>\n')
      file.close();
      rospy.loginfo(file_name + ' created!');
    
if __name__ == '__main__' :
  
  rospy.init_node('initializer')
  
  try:  
    initializer = Initializer()
    initializer.create_camera_launch()
    rospy.loginfo('Initialization completed. Press [ctrl+c] to exit.') 
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
