import csv

def main():
    data = getDataFromCSVFile();
    matlabCode = generateMatlabFunctionCode(data)
    saveMatlabCode(matlabCode)


def getDataFromCSVFile(filename = './output.csv'):
    data = {};
    columnNames = None;

    with open(filename) as csvFile:
        reader = csv.reader(csvFile, delimiter=',')
        for row in reader:
            if not columnNames:
                columnNames = row;
            else:
                for index, element in enumerate(row):
                    element = element.replace(",", ".")
                    if "e-" in element: element = '0'

                    if columnNames[index] in data:
                        data[columnNames[index]].append(element);
                    else:
                        data[columnNames[index]] = [element];

    return data

def generateMatlabFunctionCode(data):
    unitMap = {
        "crystMassFlowIntern": "(API) ml/min",
        "crystMassFlowPolymer": "ml/min",
        "hmeMassFlowAPI": "kg/h",
        "hmeMassFlowPolymer": "kg/h",
        "hmeMassFlowIntern": "kg/h",
        "dcLineMassFlowPolymer": "kg/h",
        "dcLineMassFlowIntern": "kg/h",
        "dcLineMassFlowAPI": "kg/h",
        "tabletPressMassFlowIntern": "kg/h",
        "tabletPressTabletMassFlowIntern": "tablets/h",
        "tabletPressBlenderSpeed": "%",
    }

    code = "function [value, hmeStartUpPhase, hmeTargetValue, hmeStartUpPhaseTimeOffset] = optimalSchedulingValue(time, timeOffset, fieldIdx)\n"
    code += "%% GENERATED CODE (Version: 2.0.1)\n"

    code += '%' + '\n%\t\t'.join([str(index+1) + " -> '" + key + "' - " + ("N/A" if key not in unitMap else unitMap[key]) for index, key in enumerate(data.keys())]) + "];\n"
    code += "%\tfieldNameIdxMap = [" + ", ".join(["'" + key + "'" for key in data.keys()]) + "];\n"
    code += "\tvalueMap = [";
    code += (";...\n".join([", ".join(row) for row in data.values()]))
    code += "];\n\n";

    code += "\tisValid = @(container, idx1, idx2) size(container, 1) >= idx1 && size(container, 2) >= idx2;\n\n"

    code += "\tallTimeValuesInSeconds = 60*(valueMap(1, :));\n"
    code += "\tallFutureValues = valueMap(:, (time + timeOffset) < allTimeValuesInSeconds);\n\n"

    code += "\tvalue = 0;\n"
    code += "\tif isValid(allFutureValues, fieldIdx, 1), value = allFutureValues(fieldIdx, 1); end\n\n"

    code += "\t% hmeSwitchedOn -> 6\n"
    code += "\t% hmeInOperation -> 11\n"
    code += "\t% hmeMassFlowPolymer -> 22\n"
    code += "\thmeStartUpPhase = 0;\n"
    code += "\thmeTargetValue = 0;\n"
    code += "\thmeStartUpPhaseTimeOffset = 0;\n\n"

    code += "\t%   hmeSwitchedOn_k && hmeInOperation_k+1 (due to 5min start up) && (can access target value) && (polymer value selected)\n"
    code += "\tif isValid(allFutureValues, 6, 1) && isValid(allFutureValues, 11, 2) && isValid(allFutureValues, 22, 2) && (fieldIdx == 22)\n\n"
    code += "\t\tif(allFutureValues(6, 1) && allFutureValues(11, 2) && (~allFutureValues(11, 1)))\n"
    code += "\t\t\thmeStartUpPhaseTimeOffset = 60*(allFutureValues(1, 1)-5);\n"
    code += "\t\t\thmeStartUpPhase = 1; \n"
    code += "\t\tend\n\n"

    code += "\t\thmeTargetValue = allFutureValues(22, 2);\n\n"

    code += "\tend\n"
    code += "end\n"

    return code;

def saveMatlabCode(matlabCode, filename='./optimalSchedulingValue.m'):
    with open(filename, "w+") as file:
        file.write(matlabCode);


if __name__ == "__main__":
    main()