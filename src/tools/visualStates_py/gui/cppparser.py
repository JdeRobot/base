class CPPParser():

    def __init__(self):
        pass

    @staticmethod
    def parseFunctions(funcStr):
        returnTypes = []
        funcNames = []
        codes = []
        funcExists = True
        while funcExists and len(funcStr) > 0 and funcStr.index('{') >= 0:
            funcStr = funcStr.strip()
            funcStartIndex = funcStr.index('{')
            funcSignature = funcStr[0:funcStartIndex].strip()
            returnType = funcSignature[0:funcSignature.index(' ')].strip()
            returnTypes.append(returnType)
            funcName = funcSignature[funcSignature.index(' '):].strip()
            funcNames.append(funcName)
            curlyCounter = 0
            firstCurlyFound = False
            firstCurlyIndex = None
            lastCurlyIndex = None
            for i, ch in enumerate(funcStr):

                if ch == '{':
                    curlyCounter += 1
                    firstCurlyFound = True
                    firstCurlyIndex = i
                elif ch == '}':
                    curlyCounter -= 1

                if curlyCounter == 0 and firstCurlyFound:
                    lastCurlyIndex = i
                    break
            # print(firstCurlyIndex)
            # print(lastCurlyIndex)
            # print(funcStr[firstCurlyIndex:lastCurlyIndex+1])
            codes.append(funcStr[firstCurlyIndex:lastCurlyIndex+1])
            funcExists = False

            # check whether there is any other function
            funcStr = funcStr[lastCurlyIndex+1:].strip()
            if len(funcStr) > 0 and funcStr.index('{') >= 0:
                funcExists = True

        # return returnType, funcName
        # print(returnType)
        # print(funcName)
        return returnTypes, funcNames, codes



if __name__ == '__main__':
    sampleCode = '''
  void myFunction(int a) {
    int c;
    c = a * 2;
}

void myF2 ( int b ){
    int c,d;
    c = 10;
    d = 12;
    a = c*d*b;
    return a;
    }
    
    
    
    int myfunc3() {
        int a = 12;
        int b = 324;
        int c = 0;
        c = a + b;
    }
'''

    returnTypes, funcNames, codes = CPPParser.parseFunctions(sampleCode)
    for i in range(len(returnTypes)):
        print(returnTypes[i])
        print(funcNames[i])
        print(codes[i])
    # print(returnType, funcName)