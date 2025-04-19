#!/usr/bin/env python3
# Dev. by Michael_R 2024  
# 
from time import sleep
# python bindings to libgpiod 
import gpiod
from gpiod.line import Direction,Value,Bias  # "Value", "Direction", "Bias", "Drive", "Edge", "Clock"
import gpiod.line as line 
import gpiod.line

class gpioD:
    def __init__(self,pi_chip=0):
        self.chip_str='/dev/gpiochip0' if pi_chip==0 else '/dev/gpiochip4'
        self.chip = gpiod.Chip(self.chip_str)  
        self.outputs=None
        self.inputs=None
    def close(self):
        self.chip.close()
    def config_outputs(self,ioList:list,statusList:list,biasIoList=[],biasType=[]):  # biasType=[Bias.PULL_DOWN ,Bias.AS_IS]
        config={}
        if len(biasIoList)!=0 and len(ioList)!=len(biasIoList):
            raise ValueError('pi5gpio.gpiod.config_outputs(). if par biasIoList[] and biasType[] not empty, they must be as long as ioList[] ' )
        if len(biasIoList)==0: 
            config={ioList[i]:gpiod.LineSettings(direction=Direction.OUTPUT,
                    output_value=Value.ACTIVE if statusList[i] else Value.INACTIVE,)
                                                    for i in range(len(ioList))
                                                    } # dict dynamic        
        else:
            config={ioList[i]:gpiod.LineSettings(direction=Direction.OUTPUT,
                    bias=biasType[i] if biasIoList[i] else Bias.AS_IS,
                    output_value=Value.ACTIVE if statusList[i] else Value.INACTIVE,)
                                                    for i in range(len(ioList))
                                                    } 
        self.outputs = gpiod.request_lines(self.chip_str,config)
        return
    def config_inputs(self,ioList:list,pullList:Bias): # list of Bias.PULL settings
        config={}
        config = {ioList[i]:gpiod.LineSettings(direction=Direction.INPUT,bias=pullList[i]) 
                                               for i in range(len(ioList))
                                               }        
        self.inputs=gpiod.request_lines(self.chip_str,config)
    def config_pull(self,input:int,pull:Bias):
        config = {input:gpiod.LineSettings(bias=pull)} 
    def set_outputs(self,ioList,onOffList):
        self.outputs.set_values({ioList[i]:Value.ACTIVE if onOffList[i] else Value.INACTIVE
                                        for i in range(len(ioList))
                                        })      
    def set_output(self,io,onOff):
        self.outputs.set_value(io,Value.ACTIVE if onOff else Value.INACTIVE)
    def get_line(self,gpio):
        #chip =gpiod.Chip("/dev/gpiochip4") 
        lineInfo=self.chip.get_line_info(gpio)
        lineInfo=self.chip.get_info()
        lineInfo=line.Value.INACTIVE
        lineInfo=line.Value

#usage
'''
io = gpioD()
io.config_outputs([5],[0])
io.config_outputs([12],[0])
io.config_outputs([22],[0])
io.config_outputs([5,12,22],[0,0,0])
io.config_outputs([5,12,22],[0,0,0])
io.set_outputs([5,12,22],[1,1,1])
io.set_outputs([19,22],[0,0])
io.set_output(12,1)
io.set_output(12,1)
io.set_output(17,0)
io.set_output(12,0)
io.config_inputs([13],[Bias.PULL_DOWN])
io.config_pull(13,Bias.AS_IS)
io.close()
import sys
sys.exit
'''