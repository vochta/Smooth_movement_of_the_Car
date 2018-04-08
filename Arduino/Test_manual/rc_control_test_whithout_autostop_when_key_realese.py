__author__ = 'zhengwang'

import serial
import pygame
from pygame.locals import *


class RCTest(object):

    def __init__(self):
        pygame.init()
        self.ser = serial.Serial('COM4', 9600, timeout=1)
        self.send_inst = True
        self.steer()


    def steer(self):
        window = pygame.display.set_mode((100,100))
        pygame.display.set_caption("Window")
        while self.send_inst:
            for event in pygame.event.get():
                if event.type == KEYDOWN:
                    key_input = pygame.key.get_pressed()

                    # complex orders
                    if key_input[pygame.K_UP] and key_input[pygame.K_RIGHT]:
                        print("Forward Right")
                        self.ser.write(chr(6))

                    elif key_input[pygame.K_UP] and key_input[pygame.K_LEFT]:
                        print("Forward Left")
                        self.ser.write(chr(4))

                    elif key_input[pygame.K_DOWN] and key_input[pygame.K_RIGHT]:
                        print("Reverse Right")
                        self.ser.write(chr(8))

                    elif key_input[pygame.K_DOWN] and key_input[pygame.K_LEFT]:
                        print("Reverse Left")
                        self.ser.write(chr(9))

                    # simple orders
                    elif key_input[pygame.K_UP]:
                        print("Forward")
                        self.ser.write(chr(8))

                    elif key_input[pygame.K_DOWN]:
                        print("Reverse")
                        self.ser.write(chr(2))

                    elif key_input[pygame.K_RIGHT]:
                        print("Right")
                        self.ser.write(chr(6))

                    elif key_input[pygame.K_LEFT]:
                        print("Left")
                        self.ser.write(chr(4))
                        
                    elif key_input[pygame.K_7]:
                        print("7")
                        self.ser.write(chr(7))                        
                        
                    elif key_input[pygame.K_9]:
                        print("9")
                        self.ser.write(chr(9))

                    elif key_input[pygame.K_1]:
                        print("1")
                        self.ser.write(chr(1))

                    elif key_input[pygame.K_3]:
                        print("3")
                        self.ser.write(chr(3))
                        
                    elif key_input[pygame.K_SPACE]:
                        print("Stop")
                        self.ser.write(chr(5))  
                        
                    elif key_input[pygame.K_KP_PLUS]:
                        self.ser.write(chr(9))
                        print("drive_PWM up!")
                        
                    elif key_input[pygame.K_KP_MINUS]:
                        self.ser.write(chr(7))
                        print("drive_PWM down!")      

                    elif key_input[pygame.K_a]:
                        self.ser.write(chr(3))
                        print("turn_PWM up!")
                        
                    elif key_input[pygame.K_z]:
                        self.ser.write(chr(1))
                        print("turn_PWM down!")                          
                            

                    # exit
                    elif key_input[pygame.K_x] or key_input[pygame.K_q]:
                        print 'Exit'
                        self.send_inst = False
                        self.ser.write(chr(5))
                        self.ser.close()
                        break

         #       elif event.type == pygame.KEYUP:
          #          self.ser.write(chr(5))

if __name__ == '__main__':
    RCTest()
