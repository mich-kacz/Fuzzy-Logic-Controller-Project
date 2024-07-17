from triangle import Triangle
from trapez import Trapez
from gauss import Gauss
from fuzzyFunctions import FuzzyFunctions


#import matplotlib.pyplot as plt
import numpy as np


# Fuzzy logic right wall folower class
class FuzzyLogicControllerWall:
    

    def __init__(self) -> None:
        
        #Initialization of fuzzy sets
        self.FRS = FuzzyFunctions()
        self.BRS = FuzzyFunctions()
        self.Velx = FuzzyFunctions()
        self.Velz = FuzzyFunctions()

        #Front right sensor fuzzy set
        self.FRS.addFunction("C", Trapez(0, 0, 0.25, 0.5))
        self.FRS.addFunction("M", Triangle(0.25, 0.5, 0.75))
        self.FRS.addFunction("F", Trapez(0.5, 0.75, 1, 1))

        #Back right sensor fuzzy set
        self.BRS.addFunction("C", Trapez(0, 0, 0.25, 0.5))
        self.BRS.addFunction("M", Triangle(0.25, 0.5, 0.75))
        self.BRS.addFunction("F", Trapez(0.5, 0.75, 1, 1))

        #Output x
        self.Velx.addFunction("S", Triangle(0, 0.1, 0.2))
        self.Velx.addFunction("M", Triangle(0.2, 0.3, 0.4))
        self.Velx.addFunction("F", Triangle(0.4, 0.5, 0.6))

        #Output z
        self.Velz.addFunction("R", Triangle(-0.8, -0.5, -0.1))
        self.Velz.addFunction("F", Triangle(-0.1, 0, 0.1))
        self.Velz.addFunction("L", Triangle(0.1, 0.5, 0.8))

    #Main method to run all rules and return linear and angular velocity
    def rullBase(self, sensorFR, sensorBR):
        #self._plotSets()
        frsOut = self.FRS.calculateOutput(sensorFR)
        brsOut = self.BRS.calculateOutput(sensorBR)
        firingRulesX, firingRulesZ, firingValues = self._calculateRules(frsOut, brsOut)
        velXOut, velZOut = self._calculateCentroidOutputs(firingRulesX, firingRulesZ, firingValues)

        return velXOut, velZOut
        

    #Calculation of centroids outputs and final values
    def  _calculateCentroidOutputs(self, firingRulesX, firingRulesZ, firingValues):
        velXOut = 0
        for value, key in zip(firingValues, firingRulesX):
            velXOut += value*self.Velx.peaks[key]
        velXOut = velXOut/sum(firingValues)
        velZOut = 0
        for value, key in zip(firingValues, firingRulesZ):
            velZOut += value*self.Velz.peaks[key]
        velZOut = velZOut/sum(firingValues)

        return velXOut, velZOut
        

    #Defininition and calculation of rules and firing strenghts
    def _calculateRules(self, frsOut, brsOut):
        
        firingRulesX = []
        firingRulesZ = []
        firingValues = []
        
        for rule in frsOut:
            if(frsOut[rule] > 0):
                for rule2 in brsOut:
                    if(brsOut[rule2] > 0):
                        if(rule=='C' and rule2 == 'C'):
                            firingRulesX.append('S')
                            firingRulesZ.append('L')
                            if(frsOut[rule]<brsOut[rule2]):
                                firingValues.append(frsOut[rule])
                            else:
                                firingValues.append(brsOut[rule2])

                        elif(rule=='C' and rule2 == 'M'):
                            firingRulesX.append('S')
                            firingRulesZ.append('L')
                            if(frsOut[rule]<brsOut[rule2]):
                                firingValues.append(frsOut[rule])
                            else:
                                firingValues.append(brsOut[rule2])
                        elif(rule=='C' and rule2 == 'F'):
                            firingRulesX.append('S')
                            firingRulesZ.append('L')
                            if(frsOut[rule]<brsOut[rule2]):
                                firingValues.append(frsOut[rule])
                            else:
                                firingValues.append(brsOut[rule2])
                        elif(rule=='M' and rule2 == 'C'):
                            firingRulesX.append('M')
                            firingRulesZ.append('F')
                            if(frsOut[rule]<brsOut[rule2]):
                                firingValues.append(frsOut[rule])
                            else:
                                firingValues.append(brsOut[rule2])
                        elif(rule=='M' and rule2 == 'M'):
                            firingRulesX.append('M')
                            firingRulesZ.append('F')
                            if(frsOut[rule]<brsOut[rule2]):
                                firingValues.append(frsOut[rule])
                            else:
                                firingValues.append(brsOut[rule2])
                        elif(rule=='M' and rule2 == 'F'):
                            firingRulesX.append('M')
                            firingRulesZ.append('R')
                            if(frsOut[rule]<brsOut[rule2]):
                                firingValues.append(frsOut[rule])
                            else:
                                firingValues.append(brsOut[rule2])
                        elif(rule=='F' and rule2 == 'C'):
                            firingRulesX.append('F')
                            firingRulesZ.append('R')
                            if(frsOut[rule]<brsOut[rule2]):
                                firingValues.append(frsOut[rule])
                            else:
                                firingValues.append(brsOut[rule2])
                        elif(rule=='F' and rule2 == 'M'):
                            firingRulesX.append('F')
                            firingRulesZ.append('R')
                            if(frsOut[rule]<brsOut[rule2]):
                                firingValues.append(frsOut[rule])
                            else:
                                firingValues.append(brsOut[rule2])
                        elif(rule=='F' and rule2 == 'F'):
                            firingRulesX.append('F')
                            firingRulesZ.append('R')
                            if(frsOut[rule]<brsOut[rule2]):
                                firingValues.append(frsOut[rule])
                            else:
                                firingValues.append(brsOut[rule2])

        return firingRulesX, firingRulesZ, firingValues
        
    #Function for plotting memberships functions, must be commented on real robot tests
    '''def _plotSets(self):
        values = np.linspace(0, 1, 500)
        y = []
        for x in values:
            out = self.FRS.calculateOutput(x)
            y.append(out["C"])
        plt.plot(values, y)
        y = []
        for x in values:
            out = self.FRS.calculateOutput(x)
            y.append(out["M"])
        plt.plot(values, y)
        y = []
        for x in values:
            out = self.FRS.calculateOutput(x)
            y.append(out["F"])
        plt.plot(values, y)
        plt.grid()
        plt.title("FRS")
        plt.show()
    
        y = []
        for x in values:
            out = self.BRS.calculateOutput(x)
            y.append(out["C"])
        plt.plot(values, y)
        y = []
        for x in values:
            out = self.BRS.calculateOutput(x)
            y.append(out["M"])
        plt.plot(values, y)
        y = []
        for x in values:
            out = self.BRS.calculateOutput(x)
            y.append(out["F"])
        plt.plot(values, y)
        plt.grid()
        plt.title("BRS")
        plt.show()

        y = []
        for x in values:
            out = self.Velx.calculateOutput(x)
            y.append(out["S"])
        plt.plot(values, y)
        y = []
        for x in values:
            out = self.Velx.calculateOutput(x)
            y.append(out["M"])
        plt.plot(values, y)
        y = []
        for x in values:
            out = self.Velx.calculateOutput(x)
            y.append(out["F"])
        plt.plot(values, y)
        plt.grid()
        plt.title("Velx")
        plt.show()
       
        values = np.linspace(-1, 1, 500)
        y = []
        for x in values:
            out = self.Velz.calculateOutput(x)
            y.append(out["R"])
        plt.plot(values, y)
        y = []
        for x in values:
            out = self.Velz.calculateOutput(x)
            y.append(out["F"])
        plt.plot(values, y)
        y = []
        for x in values:
            out = self.Velz.calculateOutput(x)
            y.append(out["L"])
        plt.plot(values, y)
        plt.grid()
        plt.title("Velz")
        plt.show()'''

        



def main():
    controller = FuzzyLogicControllerWall()
    print(controller.rullBase(0.45,0.45))



if __name__=="__main__":
    main()

        