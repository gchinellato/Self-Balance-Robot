#!/usr/bin/env python

# NOTE: This will not run on the Raspberry PI itself
# It is just a simple OpenGL program to visualise the orientation of the sensors
# Copyright @ http://blog.bitify.co.uk/2013/11/3d-opengl-visualisation-of-data-from.html

import pygame
import urllib
from OpenGL.GL import *
from OpenGL.GLU import *
import math
from pygame.locals import *
import socket
import logging
import Queue
from Comm.UDP.UDP_Server import UDP_ServerThread
from IMU.constants import *

SCREEN_SIZE = (800, 600)

serverUDPQueue = Queue.Queue(4)
serverThread = UDP_ServerThread(name="Thread-UDP-Server", queue=serverUDPQueue)
serverThread.start()

def resize(width, height):
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(width) / height, 0.001, 10.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    gluLookAt(1.0, 2.0, -5.0,
              0.0, 0.0, 0.0,
              0.0, 1.0, 0.0)
    
def init():
    glEnable(GL_DEPTH_TEST)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_BLEND)
    glEnable(GL_POLYGON_SMOOTH)
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST)
    glEnable(GL_COLOR_MATERIAL)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glLightfv(GL_LIGHT0, GL_AMBIENT, (0.5, 0.5, 0.5, 1.0));

def read_values():
    #Receiving UDP packets
    data = serverThread.getMessage() 
    
    #Check if is a valid message
    if data != None:
        print data
        return data 
    else:
        logging.info(None)
        return (1,1,1,1)       

def run():
    pygame.init()
    screen = pygame.display.set_mode(SCREEN_SIZE, HWSURFACE | OPENGL | DOUBLEBUF)
    resize(*SCREEN_SIZE)
    init()
    clock = pygame.time.Clock()
    cube = Cube((0.0, 0.0, 0.0), (.5, .5, .7))
    angle = 0
    
    while True:
        then = pygame.time.get_ticks()
        for event in pygame.event.get():
            if event.type == QUIT:
                return
            if event.type == KEYUP and event.key == K_ESCAPE:
                return

        values = read_values()
        roll = -float(values[1][1])*DEG_TO_RAD
        pitch = float(values[1][2])*DEG_TO_RAD
        yaw = float(values[1][3])*DEG_TO_RAD
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)


        glLineWidth(5)

        glBegin(GL_LINES)
        glColor((1., 0., 0.))
        glVertex3f(0, -1, 0)
        glVertex3f(0, 1, 0)

        glColor((0., 1., 0.))
        glVertex3f(-1, 0, 0)
        glVertex3f(1, 0, 0)

        glColor((0., 0., 1.))
        glVertex3f(-0, 0, -1.5)
        glVertex3f(0, 0, 1)

        glEnd()

        glColor((1., 1., 1.))
        glLineWidth(1)
        glBegin(GL_LINES)


        for x in range(-20, 22, 2):
            glVertex3f(x / 10., -1, -1)
            glVertex3f(x / 10., -1, 1)
        
        for x in range(-20, 22, 2):
            glVertex3f(x / 10., -1, 1)
            glVertex3f(x / 10., 1, 1)
        
        for z in range(-10, 12, 2):
            glVertex3f(-2, -1, z / 10.)
            glVertex3f(2, -1, z / 10.)

        for z in range(-10, 12, 2):
            glVertex3f(-2, -1, z / 10.)
            glVertex3f(-2, 1, z / 10.)

        for z in range(-10, 12, 2):
            glVertex3f(2, -1, z / 10.)
            glVertex3f(2, 1, z / 10.)

        for y in range(-10, 12, 2):
            glVertex3f(-2, y / 10., 1)
            glVertex3f(2, y / 10., 1)
        
        for y in range(-10, 12, 2):
            glVertex3f(-2, y / 10., 1)
            glVertex3f(-2, y / 10., -1)
        
        for y in range(-10, 12, 2):
            glVertex3f(2, y / 10., 1)
            glVertex3f(2, y / 10., -1)
        
        glEnd()
        glPushMatrix()
        glRotate(math.degrees(-float(yaw)), 0, 1, 0)
        glRotate(math.degrees(float(pitch)), 1, 0, 0)
        glRotate(math.degrees(-float(roll)), 0, 0, 1)

        cube.render()
        glPopMatrix()
        pygame.display.flip()
        #print math.degrees(float(pitch)), math.degrees(-float(roll)), math.degrees(-float(yaw))


class Cube(object):

    def __init__(self, position, color):
        self.position = position
        self.color = color

    # Cube information
    num_faces = 6

    vertices = [ (-0.5, -0.05, 1.0),
                 (0.5, -0.05, 1.0),
                 (0.5, 0.05, 1.0),
                 (-0.5, 0.05, 1.0),
                 (-0.5, -0.05, -1.0),
                 (0.5, -0.05, -1.0),
                 (0.5, 0.05, -1.0),
                 (-0.5, 0.05, -1.0) ]

    normals = [ (0.0, 0.0, +1.0),  # front
                (0.0, 0.0, -1.0),  # back
                (+1.0, 0.0, 0.0),  # right
                (-1.0, 0.0, 0.0),  # left
                (0.0, +1.0, 0.0),  # top
                (0.0, -1.0, 0.0) ]  # bottom

    vertex_indices = [ (0, 1, 2, 3),  # front
                       (4, 5, 6, 7),  # back
                       (1, 5, 6, 2),  # right
                       (0, 4, 7, 3),  # left
                       (3, 2, 6, 7),  # top
                       (0, 1, 5, 4) ]  # bottom

    def render(self):
        then = pygame.time.get_ticks()
        vertices = self.vertices
        # Draw all 6 faces of the cube
        glBegin(GL_QUADS)

        for face_no in xrange(self.num_faces):
            #Color of faces
            if face_no == 1:
                glColor(1.0, 0.0, 0.0)
            elif face_no == 2:
                glColor(0.0, 1.0, 0.0)
            elif face_no == 3:
                glColor(0.0, 0.0, 1.0)
            elif face_no == 4:
                glColor(1.0, 1.0, 0.0)
            elif face_no == 5:
                glColor(0.0, 1.0, 1.0)
            else:
                glColor(self.color)
            
            glNormal3dv(self.normals[face_no])
            v1, v2, v3, v4 = self.vertex_indices[face_no]
            glVertex(vertices[v1])
            glVertex(vertices[v2])
            glVertex(vertices[v3])
            glVertex(vertices[v4])
        glEnd()

if __name__ == "__main__":
    try:
        run()
    except KeyboardInterrupt:  
        serverThread.join()       
        print "Quit"
    
