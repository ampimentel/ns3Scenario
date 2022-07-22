import csv
import subprocess, shlex
from math import ceil
from os import path
from functools import partial
import concurrent.futures
import multiprocessing

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
    def __init__(self, equations, ident):
        self.attr = self.parseEq(equations)
        self.id = ident

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
    #print(simEntry)
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
    return out.decode('utf-8')

#receive inputs that need to be computed
def doOneMeasure(portApp, input):
    cmd = lambda x : f"./waf --run-no-build \"examples/NS3Scenario/vanet-test --numNodes=" + x["numNodes"] + " --procTime1=" + x["procTime1"] + " --nPackets1=" + x["nPackets1"] + " --procTime2=" + x["procTime2"] + " --nPackets2=" + x["nPackets2"] +" --mobTrace=" + x["seed"] + " --scenario=" + x["scenario"]+ "\""
    simResutlt = cmdFunc(cmd(input))
    output = getSimValues(pattern, simResutlt, portApp)
    return {**input, **output}



def measureAndWrite(portsApp, folder, filename, measure):
    val = doOneMeasure(portsApp, measure)
    writeMeasure(val, folder, filename)

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

def getInputsScenario1(scheme, numKeywords):
    pT, nP = scheme.getAlgMeasures("enc", numKeywords, nbrLin=50)
    return pT * 1000, ceil(nP / 65)


def getInputsScenario2(scheme, numKeywords, numDisj, nLin):
    #getAlgMeasures(self, algName, nbrKey, nbrDisj=0, nbrLin=1)
    pT1, nP1 = scheme.getAlgMeasures("gT", numKeywords, numDisj)
    pT2, _ = scheme.getAlgMeasures("tst", numKeywords, numDisj, nLin)
    _, sizeEnc = scheme.getAlgMeasures("enc", numKeywords, nbrLin=nLin)
    nP2 = ceil(sizeEnc / 650)
    nP1 = ceil(nP1 / 65)
    return {"nPackets1":str(nP1), "procTime1":str(pT1*1000),  "nPackets2":str(nP2), "procTime2":str(pT2*1000)}


def writeMeasures(measures, folder, filename):
    keys = measures[0].keys()
    print("MEASURES")
    print(keys)
    if not path.exists(folder + "/" + filename + ".csv"):
        with open(folder + "/" +filename + ".csv", mode="w") as file:
            dict_writer = csv.DictWriter(file, keys)
            dict_writer.writeheader()
            dict_writer.writerows(measures)
    else:
        with open(folder + "/" + filename + ".csv", mode="a") as file:
            dict_writer = csv.DictWriter(file, keys)
            dict_writer.writerows(measures)

def main():
    
    portsApp = {48: "1", 15: "2"}
    schemePlain = {"gT_T": "0.001", "enc_T" : "0.001", "tst_T" : "0.000001", "gT_S" : "1", "enc_S": "0.008*x"}
    scheme1 = {"gT_T": "0.01*x2 + 0.003*x", "enc_T" : "0.003*x+0.027", "tst_T" : "0.105*y+0.113", "gT_S" : "0.337*y+0.533", "enc_S": "0.074*x + 2.078"}
    scheme2 = {"gT_T": "0.004*x", "enc_T" : "0.057*x+0.014", "tst_T" : "0.054*y+0.057", "gT_S" : "0.179*y+0.447", "enc_S": "0.781*x + 0.307"}
    sePlain = SEScheme(schemePlain, 0)
    seScheme1 = SEScheme(scheme1, 1)
    seScheme2 = SEScheme(scheme2, 2)
    scheme = [sePlain, seScheme1, seScheme2]
    numNodes = [100]
    numDisjunctions = [1, 2]
    numKeywords = [20, 100]
    numLines = [50,1000]
    seed = [1 ,2, 3]
    inputs = []

    for sc in scheme:
        for nN in numNodes:
            for nL in numLines:
                for nK in numKeywords:
                    for nD in numDisjunctions:
                        for s in seed:
                            parm = getInputsScenario2(sc, nK, nD, nL)
                            inp = {"scheme" : str(sc.id), "numNodes" : str(nN), "numKeywords" : str(nK), "seed": str(s), "scenario" : str(2), "numLines" : nL, "numDisjunctions": nD}
                            inputs.append({**inp, **parm})
    
    doneMeasures = readMeasures("measuresTest","test5")
    alreadyMeasure(doneMeasures, inputs)
    print(inputs)
    funcThread2 = partial(doOneMeasure, portsApp)
    with concurrent.futures.ThreadPoolExecutor(max_workers=20) as executor:
        results = executor.map(funcThread2, inputs)
    
    print("Hello changes")
    lstRes = list(results)
    print(lstRes)

    if len(lstRes) > 0:
        print("HI")
        writeMeasures(lstRes,"measuresTest","test5" )
main()
