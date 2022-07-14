import csv
from curses import nl
import subprocess, shlex
from math import ceil
from os import path, remove

from matplotlib.pyplot import table


class SEScheme:
    
    def parseEq(self, dictEquation):
        pars = dict()
        for name in dictEquation:
            eqs = dictEquation[name].replace(" ", "").split("+")
            for eq in eqs:
                if "*" in eq:
                    nbr, par = eq.split("*", 1)
                else:
                    nbr, par = eq, "c"
                pars[name + "_" + par] = float(nbr)
        return pars
        
    #initialize an algorithm with attributes 
    def __init__(self, equations):
        attr = self.parseEq(equations)

    def getAttrs(self, algName, values):
        inf = algName + "_" 
        return [self.attr[inf + val] if inf + val in self.attr else 0 for val in values] 

    def dotProd(self, lst1, lst2):
        return sum(a*b for a, b in zip(lst1, lst2))
    
    #computes time and space that a given algorithm will take
    def getAlgMeasures(self, algName, nbrKey, nbrDisj=0, nbrLin=1):
        inputs = [nbrKey**2, nbrKey, nbrDisj**2, nbrDisj, 1]
        tAttr = self.getAttrs(algName + "_T", ["x2", "x", "y2", "y", "c"])
        sAttr = self.getAttrs(algName + "_S", ["x2", "x", "y2", "y", "c"])
        time = self.dotProd(tAttr, inputs)
        space = self.dotProd(sAttr, inputs)
        if algName == "enc" or algName == "tst":
            time *= nbrLin
            space *= nbrLin
        return time, space

pattern = "\n===BEGIN SIMULATION===\n"

def simGetInt(valId, simOutput, extra = ""):
    rightPartition = simOutput.split(valId)[1]
    return {valId[:-2] + extra: int(rightPartition.split("\n")[0])}


def getSimValues(pattern, simEntry, portsApp):
    simResults = simEntry.split(pattern)[1]
    simCleanEntry=  simResults.split("Info on App with destination port:")[1:]
    lostPackets, rcvPackets, avgDelay = {}, {}, {}
    for sim in simCleanEntry:
        port = int(sim.split("\n")[0])
        lostPackets.update(simGetInt("Total Lost Packets: ", sim, portsApp[port]))
        rcvPackets.update(simGetInt("Total Received Packets: ", sim, portsApp[port]))
        avgDelay.update(simGetInt("Average Delay (ms): ", sim, portsApp[port]))
    return {**lostPackets, **rcvPackets, **avgDelay}

def cmdFunc(cmd):
    print(cmd)
    process = subprocess.Popen(shlex.split(cmd), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, _ = process.communicate()
    print(out.decode('utf-8'))
    return out.decode('utf-8')

#receive inputs that need to be computed
def doOneMeasure(input, portApps):
    cmd = lambda x : f"./waf --run \"scratch/vanet-scenario2 --numNodes=" + x["numNodes"] + " --procTime1=" + x["procTime1"] + " --nPackets1=" + x["nPackets1"] + " --procTime2=" + x["procTime2"] + " --nPackets2=" + x["nPackets2"] +" --mobTrace=" + x["seed"] + " --scenario=" + x["scenario"]+ "\""
    simResutlt = cmdFunc(cmd(input))
    output = getSimValues(pattern, simResutlt, portApps)    
    return {**input, **output}


def writeMeasure(measure, folder, filename):
    if not path.exists(folder + "/" + filename + ".csv"):
        with open(folder + "/" +filename + ".csv", mode="w") as file:
            writer = csv.DictWriter(file, fieldnames = measure.keys())
            writer.writeheader()
    with open(folder + "/" + filename + ".csv", mode="a") as file:
        writer = csv.DictWriter(file, fieldnames = measure.keys())
        writer.writerow(measure)

def measureAndWrite(input, portsApp, folder, filename):
    measure = doOneMeasure(input, portsApp)
    writeMeasure(measure, folder, filename)

def readMeasures(folder, filename):
    if path.exists(folder + "/" + filename + ".csv"):
        with open(folder + "/" + filename + ".csv", mode="r") as file:
            writer = csv.DictReader(file)
            return list(writer)
    return []

#removes measures already done from inputs
def alreadyMeasure(doneMeasures, inputs):     
    for g in doneMeasures:
        for inp in inputs:
            if inp.items() <= g.items():
                inputs.remove(inp)
                break
    #for g in doneMeasures:
    #    if g in inputs:
    #        inputs.remove(g)

def getPacktesAndTime(scheme, nK):
    if scheme == 0:
        nPackets =  1
        processingTime = 10
    elif scheme == 1:
        nPackets =  ceil(50*(0.074*nK + 2.078) / 65.0)
        processingTime = 3 * nK + 27
    else: 
        nPackets =  ceil(50*(0.781*nK + 0.307) / 65.0)
        processingTime = 57 * nK + 14
    return nPackets, processingTime, nPackets, processingTime
    

def getInputsScenario 1(scheme, numKeywords, numDisj, )

def main():
    
    portsApp = {48: "1", 15: "2"}
    numNodes = [10]#, 60, 100]#, 5, 10, 100, 1000, 100000]#, 10, 50, 100, 1000, 10000]
    numKeywords = [1, 2]#, 10, 50]#, 5, 10, 100]
    scheme = [0]
    seed = [1]
    inputs = []
    scheme1 = {"gT_T": "0.01x2 + 0.003x", "enc_T" : "0.003x+0.027", "tst_T" : "0.105y+0.113", "gT_S" : "0.337y+0.533", "enc_S": "0.074x + 2.078"}
    scheme2 = {"gT_T": "0.004x", "enc_T" : "0.057x+0.014", "tst_T" : "0.054y+0.057", "gT_S" : "0.179+0.447", "enc_S": "0.781x + 0.307"}
    seScheme1 = SEScheme(scheme1)
    seScheme2 = SEScheme(scheme2)
    for sc in scheme:
        for nN in numNodes:
            for nK in numKeywords:
                for s in seed:
                    nP1, pT1, nP2, pT2 = getPacktesAndTime(sc, nK)
                    inputs.append({"scheme" : str(sc), "numNodes" : str(nN), "numKeywords" : str(nK), "nPackets1":str(nP1), "procTime1":str(pT1),  "nPackets2":str(nP2), "procTime2":str(pT2), "seed": str(s), "scenario" : str(2)})
    
    doneMeasures = readMeasures("measuresTest","test1")
    print(doneMeasures)
    alreadyMeasure(doneMeasures, inputs)
    print(inputs)
    for measure in inputs:
        measureAndWrite(measure, portsApp, "measuresTest","test1")
main()






#considerações:
#não permite aind recuperar os elementos desencriptados
#o servidor sabe parte da query: "ID and Info"
#podem ser usados mecanismos de caching para acelerar as querys: associar trapdoors a entradas da dB
#podemos ainda usar estratégias mistas: guardar apenas alguns valores críticos (timestamp, velocidade, Enc(ID))
