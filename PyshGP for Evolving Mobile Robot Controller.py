"""start supervisor."""


from controller import Camera, DistanceSensor, Motor, Robot, TouchSensor, Supervisor
import pyshgp.push.instruction_set
import pyshgp.gp.genome
import pyshgp.gp.individual
import pyshgp.gp.population
import pyshgp.gp.evaluation
import pyshgp.push.interpreter
from pyshgp.push.atoms import *
import pyshgp.push.types
import pyshgp.monitoring
import logging
import pyshgp.gp.search
import pyshgp.gp.estimators
import pyshgp.gp.selection
import numpy as np
import pyshgp.gp.variation
from pyshgp.push.instruction import SimpleInstruction
import time
from pyshgp.push.type_library import PushTypeLibrary
from pyshgp.push.types import PushInt
from  pyshgp.push.types import PushFloat
import numpy as np
from bisect import insort_left
import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.pyplot as plt
import pickle


TIME_STEP = 48
supervisor = Supervisor()
node = supervisor.getSelf()



#get motors
RightForelegJ1 = supervisor.getMotor('PRM:/r4/c1-Joint2:41')
RightForelegJ2 = supervisor.getMotor('PRM:/r4/c1/c2-Joint2:42')
RightForelegJ3 = supervisor.getMotor('PRM:/r4/c1/c2/c3-Joint2:43')

RightHindlegJ1 = supervisor.getMotor('PRM:/r5/c1-Joint2:51')
RightHindlegJ2 = supervisor.getMotor('PRM:/r5/c1/c2-Joint2:52')
RightHindlegJ3 = supervisor.getMotor('PRM:/r5/c1/c2/c3-Joint2:53')

LeftForelegJ1 = supervisor.getMotor('PRM:/r2/c1-Joint2:21')
LeftForelegJ2 = supervisor.getMotor('PRM:/r2/c1/c2-Joint2:22')
LeftForelegJ3 = supervisor.getMotor('PRM:/r2/c1/c2/c3-Joint2:23')

LeftHindlegJ1 = supervisor.getMotor('PRM:/r3/c1-Joint2:31')
LeftHindlegJ2 = supervisor.getMotor('PRM:/r3/c1/c2-Joint2:32')
LeftHindlegJ3 = supervisor.getMotor('PRM:/r3/c1/c2/c3-Joint2:33')


RightForelegJ1Ins = []
RightForelegJ2Ins = []
RightForelegJ3Ins = []
            
RightHindlegJ1Ins = []
RightHindlegJ2Ins = []
RightHindlegJ3Ins = []
        
LeftForelegJ1Ins = []
LeftForelegJ2Ins = []
LeftForelegJ3Ins = []
        
LeftHindlegJ1Ins = []
LeftHindlegJ2Ins = []
LeftHindlegJ3Ins = []
#set velocity of joints

POP_SIZE = 10
GENERATIONS = 50

BESTERRLIST = []
POP = pyshgp.gp.population.Population()
POPINS = []




inst = pyshgp.push.instruction_set.InstructionSet()

inst.register_core_by_stack({"float"}, exclude_stacks={None})

#inst.register_core()

lit = [-2.0,-1.9,-1.8,-1.7,-1.6,-1.5,-1.4,-1.3,-1.2,-1.1,-1.0,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2.0]
 
erc = []
genome = pyshgp.gp.genome.GeneSpawner(inst,lit,erc)



selector = pyshgp.gp.selection.Tournament(7)

addition = pyshgp.gp.variation.AdditionMutation(0.09)
alternation = pyshgp.gp.variation.Alternation(0.09)

deletion = pyshgp.gp.variation.DeletionMutation(0.0826)

pipeline = pyshgp.gp.variation.VariationPipeline([addition,deletion,alternation])


searchConfig = pyshgp.gp.search.SearchConfiguration(evaluator = eval,spawner = genome, variation = pipeline, selection = selector, population_size = POP_SIZE, max_generations = GENERATIONS)
search = pyshgp.gp.search.GeneticAlgorithm(searchConfig)







def moveMotor(motor, position):
    
    if(position < motor.getMinPosition() or position > motor.getMaxPosition()):
        pass
    else: 
        motor.setPosition(position)



def output(pop):
    out = []
    interpreterConfig = pyshgp.push.interpreter.PushInterpreterConfig()
    interpreter = pyshgp.push.interpreter.PushInterpreter(inst,interpreterConfig)
    for i in range(len(pop)):
        program = pop[i].program
        output = interpreter.run(program,lit,['float']*120)

        out.append(output)
        
  
            
    return out

def scanIns(ins):
    err = 0
    for i in range(len(ins)):
        if type(ins[i]) ==type(0.0):
            pass
        else:
            err +=1
    return err

def primeIndividual(instructions):
    global RightForelegJ1Ins
    global RightForelegJ2Ins
    global RightForelegJ3Ins
                
    global RightHindlegJ1Ins
    global RightHindlegJ2Ins
    global RightHindlegJ3Ins
            
    global LeftForelegJ1Ins
    global LeftForelegJ2Ins
    global LeftForelegJ3Ins
          
    global LeftHindlegJ1Ins
    global LeftHindlegJ2Ins
    global LeftHindlegJ3Ins
    
    RightForelegJ1Ins = []
    RightForelegJ2Ins = []
    RightForelegJ3Ins = []
                
    RightHindlegJ1Ins = []
    RightHindlegJ2Ins = []
    RightHindlegJ3Ins = []
            
    LeftForelegJ1Ins = []
    LeftForelegJ2Ins = []
    LeftForelegJ3Ins = []
          
    LeftHindlegJ1Ins = []
    LeftHindlegJ2Ins = []
    LeftHindlegJ3Ins = []
    count = -1
   
    for i in range(len(instructions)):
        count+=1
        if count==0:
            RightForelegJ1Ins.append(instructions[i])
        if count==1:
            LeftForelegJ1Ins.append(instructions[i])
        if count==2:
            RightHindlegJ1Ins.append(instructions[i])
        if count==3:
            LeftHindlegJ1Ins.append(instructions[i])
        if count==4:
            RightForelegJ2Ins.append(instructions[i])
        if count==5:
            LeftForelegJ2Ins.append(instructions[i])
        if count==6:
            RightHindlegJ2Ins.append(instructions[i])
        if count==7:
            LeftHindlegJ2Ins.append(instructions[i])
        if count==8:
            RightForelegJ3Ins.append(instructions[i])
        if count==9:
            LeftForelegJ3Ins.append(instructions[i])
        if count==10:
            RightHindlegJ3Ins.append(instructions[i])
        if count==11:
            count = -1
            LeftHindlegJ3Ins.append(instructions[i])
    


search.init_population()


ind_count = 0
pop_count = 0
gen_count = 0
currerr = 0
#currerry = 0

POPINS = output(search.population)

primeIndividual(POPINS[0])
origin = node.getPosition()

print(origin)
search.best_seen = search.population[0]

popdict = {}

while supervisor.step(TIME_STEP) != -1:

    
    try:
        moveMotor(RightForelegJ1,float(RightForelegJ1Ins[ind_count]))
        moveMotor(LeftForelegJ1,float(LeftForelegJ1Ins[ind_count]))
        moveMotor(RightHindlegJ1,float(RightHindlegJ1Ins[ind_count]))
        moveMotor(LeftHindlegJ1,float(LeftHindlegJ1Ins[ind_count]))
        
        moveMotor(RightForelegJ2,float(RightForelegJ2Ins[ind_count]))
        moveMotor(RightHindlegJ2,float(RightHindlegJ2Ins[ind_count]))
        moveMotor(LeftForelegJ2,float(LeftForelegJ2Ins[ind_count]))
        moveMotor(LeftHindlegJ2,float(LeftHindlegJ2Ins[ind_count]))
        
        moveMotor(RightForelegJ3,float(RightForelegJ3Ins[ind_count]))
        moveMotor(RightHindlegJ3,float(RightHindlegJ3Ins[ind_count]))
        moveMotor(LeftForelegJ3,float(LeftForelegJ3Ins[ind_count]))
        moveMotor(LeftHindlegJ3,float(LeftHindlegJ3Ins[ind_count]))
    except:
        pass            
    ind_count += 1        
    supervisor.step(100)
    if ind_count == 15:
        currpos = node.getPosition()
        currerr += 1/(currpos[0] - origin[0])
        
        
        if currerr < 0:
           currerr = 100000
        #currerry += (10*origin[1])/currpos[1]
        currerr += scanIns(POPINS[pop_count])
       
        
     

       
        
        
          
        search.population[pop_count].error_vector = np.array([currerr])
        print("Error {0}".format(search.population[pop_count].error_vector))
        
            
        popdict[search.population[pop_count]] = search.population[pop_count].error_vector[0]
           
        print("Individual: {0} , Generation {1}\n".format(pop_count+1, gen_count+1))
        
        if (search.population[pop_count].total_error < search.best_seen.total_error) :
            
            BEST = POPINS[pop_count]
            search.best_seen = search.population[pop_count]
            
        currerr = 0
        #currerry = 0
        ind_count = 0
        pop_count += 1
       
        if pop_count < POP_SIZE:
            primeIndividual(POPINS[pop_count])
        supervisor.simulationReset()
        
        
    if (pop_count == POP_SIZE):
       
        
        popdict = {k: v for k, v in sorted(popdict.items(), key=lambda item: item[1])}
        
        search.population.evaluated = list(popdict.keys())
        
     
        print("Best Error in population{0}".format(search.population.best().error_vector))
        
        print("Best seen so far {0}\n".format(search.best_seen.error_vector))
        
        
        
        
        BESTERRLIST.append(np.sum(search.population.best().error_vector))
        
        search.population.unevaluated = []
        popdict = {}
        
       
        
        pop_count = 0 
        gen_count  += 1
        
           
        search.step()
        
        POPINS = output(search.population)
       
      
      
        
     
            
    if gen_count == GENERATIONS:
        plt.plot(BESTERRLIST)
        plt.show()
        
        with open('best_program', 'w') as best_program:
            best_program.write(str(search.best_seen.program))
        with open('best_file.txt', 'w') as best_file:
            best_file.write(str((output([search.best_seen])[0])))
        break
        
      
       




