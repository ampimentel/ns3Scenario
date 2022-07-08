
import csv
import subprocess, shlex
from functools import partial
import json
import argparse
import random
from math import ceil
from os import path

pattern = "\n===BEGIN SIMULATION===\n"

def simGetInt(valId, simOutput):
    rightPartition = simOutput.split(valId)[1]
    return {valId: int(rightPartition.split("\n")[0])}

def getSimValues(pattern, simEntry):
    simCleanEntry = simEntry.split(pattern)[1]
    lostPackets = simGetInt("Total Lost Packets:", simCleanEntry)
    rcvPackets = simGetInt("Total Received Packets:", simCleanEntry)
    avgDelay = simGetInt("Average Delay (ms):", simCleanEntry)
    return {**lostPackets, **rcvPackets, **avgDelay}

def cmdFunc(cmd):
    process = subprocess.Popen(shlex.split(cmd), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, _ = process.communicate()
    return out.decode('utf-8')

#receive inputs that need to be computed
def doOneMeasure(input):
    cmd = lambda x : f"./waf --run \"scratch/vanet-test --numNodes=" + x["numNodes"] + " --procTime=" + x["procTime"] + " --nPackets=" + x["nPackets"] +" --mobTrace=" + x["seed"] + "\""
    simResutlt = cmdFunc(cmd(input))
    output = getSimValues(pattern, simResutlt)    
    return {**input, **output}


def writeMeasure(measure, folder, filename):
    if not path.exists(folder + "/" + filename + ".csv"):
        with open(folder + "/" +filename + ".csv", mode="w") as file:
            writer = csv.DictWriter(file, fieldnames = measure.keys())
            writer.writeheader()
    with open(folder + "/" + filename + ".csv", mode="a") as file:
        writer = csv.DictWriter(file, fieldnames = measure.keys())
        writer.writerow(measure)

def measureAndWrite(input, folder, filename):
    measure = doOneMeasure(input)
    writeMeasure(measure, folder, filename)

def readMeasures(folder, filename, dictEntries):
    if path.exists(folder + "/" + filename + ".csv"):
        with open(folder + "/" + filename + ".csv", mode="r") as file:
            writer = csv.DictReader(file)
            return [{en:line[en] for en in dictEntries} for line in writer]
    return []

def alreadyMeasure(doneMeasures, inputs): 
    for g in doneMeasures:
        if g in inputs:
            inputs.remove(g)

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
    return nPackets, processingTime
    
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', required = True, type = int, help='Seed to random generate data')
    parser.add_argument('--pathMeasures', default = "measures", help='Path to Measures Folder')

    args = parser.parse_args()

    numNodes = [10]#, 60, 100]#, 5, 10, 100, 1000, 100000]#, 10, 50, 100, 1000, 10000]
    numKeywords = [1]#, 10, 50]#, 5, 10, 100]
    scheme = [0, 1]
    seed = [1]
    inputs = []
    for sc in scheme:
        for nN in numNodes:
            for nK in numKeywords:
                for s in seed:
                    nP, pT = getPacktesAndTime(sc, nK)
                    inputs.append({"scheme" : str(sc), "numNodes" : str(nN), "numKeywords" : str(nK), "nPackets":str(nP), "procTime":str(pT), "seed": str(s)})
    
    doneMeasures = readMeasures("measuresTest","test1", ["scheme", "numNodes", "numKeywords", "nPackets", "procTime", "seed"])
    alreadyMeasure(doneMeasures, inputs)
    print(inputs)
    for measure in inputs:
        measureAndWrite(measure, "measuresTest","test1")
main()

#o que falta: 
#1. Mesmo mapa, carros em diferentes posições (x)
#2. Tempo de simulação?
#3. Paralelizar? 
#4. DockerFile 
#5. Upload no server e bombar!!!!!! 




#considerações:
#não permite aind recuperar os elementos desencriptados
#o servidor sabe parte da query: "ID and Info"
#podem ser usados mecanismos de caching para acelerar as querys: associar trapdoors a entradas da dB
#podemos ainda usar estratégias mistas: guardar apenas alguns valores críticos (timestamp, velocidade, Enc(ID))
